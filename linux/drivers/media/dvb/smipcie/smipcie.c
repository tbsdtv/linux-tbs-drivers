/*
    SMI PCIe driver for DVBSky cards.
    Copyright (C) 2014 Max Nibble <nibble.max@gmail.com>

*/
#include "smipcie.h"
#include "m88rs6000.h"
#include "m88ds3103.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activates frontend debugging (default:0)");

#define dprintk(args...) \
	do { \
		if (debug) \
			printk(KERN_INFO "smi_pcie: " args); \
	} while (0)


DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static int smi_hw_init(struct smi_dev *dev)
{
	u32 port_mux, port_ctrl, int_stat;
	dprintk("%s\n", __func__);
	
	/* set port mux.*/
	port_mux = smi_read(MUX_MODE_CTRL);
	port_mux &= ~(rbPaMSMask);
	port_mux |= rbPaMSDtvNoGpio;
	port_mux &= ~(rbPbMSMask);
	port_mux |= rbPbMSDtvNoGpio;
	port_mux &= ~(0x0f0000);
	port_mux |= 0x50000;	
	smi_write(MUX_MODE_CTRL, port_mux);
	
	/* set DTV register.*/
	/* Port A */
	port_ctrl = smi_read(VIDEO_CTRL_STATUS_A);
	port_ctrl &= ~0x01;
	smi_write(VIDEO_CTRL_STATUS_A, port_ctrl);
	port_ctrl = smi_read(MPEG2_CTRL_A);
	port_ctrl &= ~0x40;
	port_ctrl |= 0x80;
	smi_write(MPEG2_CTRL_A, port_ctrl);
	/* Port B */
	port_ctrl = smi_read(VIDEO_CTRL_STATUS_B);
	port_ctrl &= ~0x01;
	smi_write(VIDEO_CTRL_STATUS_B, port_ctrl);
	port_ctrl = smi_read(MPEG2_CTRL_B);
	port_ctrl &= ~0x40;
	port_ctrl |= 0x80;
	smi_write(MPEG2_CTRL_B, port_ctrl);
	
	/* disable and clear interrupt.*/
	smi_write(MSI_INT_ENA_CLR, ALL_INT);	
	int_stat = smi_read(MSI_INT_STATUS);
	smi_write(MSI_INT_STATUS_CLR, int_stat);
	
	/* reset demod.*/
	smi_clear(PERIPHERAL_CTRL, 0x0303);
	msleep(50);
	smi_set(PERIPHERAL_CTRL, 0x0101);	    	
	return 0;	
}

static void smi_ir_enableInterrupt(struct smi_rc *ir)
{
	struct smi_dev *dev = ir->dev;
	smi_write(MSI_INT_ENA_SET, IR_X_INT);
}
static void smi_ir_disableInterrupt(struct smi_rc *ir)
{
	struct smi_dev *dev = ir->dev;
	smi_write(MSI_INT_ENA_CLR, IR_X_INT);    
}
static void smi_ir_clearInterrupt(struct smi_rc *ir)
{
	struct smi_dev *dev = ir->dev;
	smi_write(MSI_INT_STATUS_CLR, IR_X_INT);
}
static void smi_ir_start(struct smi_rc *ir)
{
	struct smi_dev *dev = ir->dev;
	
	smi_write(IR_Idle_Cnt_Low, 0x00140070);
	/* smi_write(IR_Idle_Cnt_Low, 0x003200c8); */
	msleep(2);
	smi_set(IR_Init_Reg, 0x90);
	
	smi_ir_enableInterrupt(ir);
	/* tasklet_enable(&ir->tasklet);*/
}
static void smi_ir_stop(struct smi_rc *ir)
{
	struct smi_dev *dev = ir->dev;
	
	/* tasklet_disable(&ir->tasklet);*/
	smi_ir_disableInterrupt(ir);
	smi_clear(IR_Init_Reg, 0x80);
}
/*
static int smi_ir_open(struct rc_dev *rc)
{
	struct smi_rc *ir = rc->priv;
	if(!ir)
		return -ENODEV;
	
	if(ir->users++ == 0)
		smi_ir_start(ir);
	return 0;
}
static void smi_ir_close(struct rc_dev *rc)
{
	struct smi_rc *ir = rc->priv;
	if(ir) {
		--ir->users;
		if(!ir->users)
			smi_ir_stop(ir);	
	}
}
*/

#define BITS_PER_COMMAND 14
#define GROUPS_PER_BIT   2
#define IR_RC5_MIN_BIT	36
#define IR_RC5_MAX_BIT	52
static u32 smi_decode_rc5(u8 *pData, u8 size)
{
	u8 index, current_bit, bit_count;
	u8 group_array[BITS_PER_COMMAND * GROUPS_PER_BIT + 4];
	u8 group_index = 0;
	u32 command = 0xFFFFFFFF;
	
	group_array[group_index++] = 1;
	
	for(index = 0; index < size; index++) {
		
		current_bit = (pData[index] & 0x80) ? 1 : 0;
		bit_count = pData[index] & 0x7f;		
		
		/*dprintk("index[%d]: bit = %d, count = %d\n", index, current_bit, bit_count);*/
		if((current_bit == 1) && (bit_count >= 2*IR_RC5_MAX_BIT + 1)) {
			goto process_code;	
		} else if((bit_count >= IR_RC5_MIN_BIT) && (bit_count <= IR_RC5_MAX_BIT)) {
		                group_array[group_index++] = current_bit;
            	} else if ((bit_count > IR_RC5_MAX_BIT) && (bit_count <= 2*IR_RC5_MAX_BIT)) {
				group_array[group_index++] = current_bit;
				group_array[group_index++] = current_bit;
		} else {
			goto invalid_timing;
		}
		if(group_index >= BITS_PER_COMMAND*GROUPS_PER_BIT)
		 	goto process_code;
		 	
		 if((group_index == BITS_PER_COMMAND*GROUPS_PER_BIT - 1) 
		 	&& (group_array[group_index-1] == 0)) {
		 	group_array[group_index++] = 1;
		 	goto process_code;
		}
	}
		
process_code:
	if(group_index == (BITS_PER_COMMAND*GROUPS_PER_BIT-1)) {
		group_array[group_index++] = 1;
	}
	
	/*dprintk("group index = %d\n", group_index);*/
	if(group_index == BITS_PER_COMMAND*GROUPS_PER_BIT) {
    		command = 0;
    		for(index = 0; index < (BITS_PER_COMMAND*GROUPS_PER_BIT); index = index + 2) {
        		/*dprintk("group[%d]:[%d]->[%d]\n", index/2, group_array[index], group_array[index+1]);*/
        		if((group_array[index] == 1) && (group_array[index+1] == 0)) {
            			command |= ( 1 << (BITS_PER_COMMAND - (index/2) - 1));
        		} else if((group_array[index] == 0) && (group_array[index+1] == 1)) {    

        		} else {
        			command = 0xFFFFFFFF;
				goto invalid_timing;
			}
        	}
    	}
    		
invalid_timing:
	/*dprintk("rc5 code = %x\n", command);*/
	return command;
}

/* static void smi_ir_decode(unsigned long data) */
static void smi_ir_decode(struct work_struct *work)
{
	/* struct smi_rc *ir = (struct smi_rc *)data; */
	struct smi_rc *ir = container_of(work, struct smi_rc, work);
	struct smi_dev *dev = ir->dev;
	struct rc_dev *rc_dev = ir->rc_dev;
/*	int ret;
	struct ir_raw_event ev; */
	u32 dwIRControl, dwIRData, dwIRCode, scancode;
	u8 index, ucIRCount, readLoop, rc5_command, rc5_system, toggle;
		
	dwIRControl = smi_read(IR_Init_Reg);
	/*dprintk("IR_Init_Reg = 0x%08x\n", dwIRControl);*/
	if( dwIRControl & rbIRVld) {
               	ucIRCount = (u8) smi_read(IR_Data_Cnt);
               	/*dprintk("ir data count=%d\n", ucIRCount);*/           	
               	if(ucIRCount < 4) {
               		goto end_ir_decode;
               	}
               	         		
               	readLoop = ucIRCount/4;
               	if(ucIRCount % 4)
               		readLoop += 1;
               	for(index = 0; index < readLoop; index++) {
               		dwIRData = smi_read(IR_DATA_BUFFER_BASE + (index*4));
               		/*dprintk("ir data[%d] = 0x%08x\n", index, dwIRData);*/
                		
               		ir->irData[index*4 + 0] = (u8)(dwIRData);
               		ir->irData[index*4 + 1] = (u8)(dwIRData >> 8);
               		ir->irData[index*4 + 2] = (u8)(dwIRData >> 16);
               		ir->irData[index*4 + 3] = (u8)(dwIRData >> 24);             		
               	}               	
               	dwIRCode = smi_decode_rc5(ir->irData, ucIRCount);
               	
		if (dwIRCode != 0xFFFFFFFF) {
			dprintk("rc code: %x \n", dwIRCode);
			rc5_command = dwIRCode & 0x3F;
			rc5_system = (dwIRCode & 0x7C0) >> 6;
			toggle = (dwIRCode & 0x800) ? 1 : 0;		
			scancode = rc5_system << 8 | rc5_command;
			rc_keydown(rc_dev, scancode, toggle);
		}
		
		/*
               	ir_raw_event_reset(rc_dev);
               	init_ir_raw_event(&ev);
		ev.pulse = 1;
		ev.duration = RC5_UNIT;
		ir_raw_event_store(rc_dev, &ev);               	
               	for(index = 0; index < ucIRCount; index++) {
			ev.pulse = (ir->irData[index] & 0x80)  ? 1 : 0;
			ev.duration = 20000 * (ir->irData[index] & 0x7f);
			ir_raw_event_store(rc_dev, &ev);
			//ir_raw_event_store_with_filter(rc_dev, &ev);             		
               	}
               	//ir_raw_event_set_idle(rc_dev, true);
               	ir_raw_event_handle(rc_dev); */
	}
end_ir_decode:	
	smi_set(IR_Init_Reg, 0x04);
	smi_ir_enableInterrupt(ir);	
}
static void smi_ir_irq(struct smi_rc *ir, u32 int_status)
{
	if(int_status & IR_X_INT) {
		smi_ir_disableInterrupt(ir);
		smi_ir_clearInterrupt(ir);
		/* tasklet_schedule(&ir->tasklet);*/
		schedule_work(&ir->work);
	}
}
static int smi_ir_init(struct smi_dev *dev)
{
	int ret;
	struct rc_dev *rc_dev;
	struct smi_rc *ir = &dev->ir;
	
	dprintk("%s\n", __func__);
	rc_dev = rc_allocate_device();
	if(!rc_dev)
		return -ENOMEM;
		
	/* init input device */
	snprintf(ir->input_name, sizeof(ir->input_name), "IR (%s)", dev->info->name);
	snprintf(ir->input_phys, sizeof(ir->input_phys), "pci-%s/ir0", pci_name(dev->pci_dev));		
	
	rc_dev->driver_name = "SMI_PCIe";
	rc_dev->input_phys = ir->input_phys;
	rc_dev->input_name = ir->input_name;
	rc_dev->input_id.bustype = BUS_PCI;
	rc_dev->input_id.version = 1;
	rc_dev->input_id.vendor = dev->pci_dev->subsystem_vendor;
	rc_dev->input_id.product = dev->pci_dev->subsystem_device;
	rc_dev->dev.parent = &dev->pci_dev->dev;
	
	rc_dev->driver_type = RC_DRIVER_SCANCODE;	
	rc_dev->map_name = RC_MAP_DVBSKY;
	/*
	rc_dev->driver_type = RC_DRIVER_IR_RAW;
	rc_set_allowed_protocols(rc_dev, RC_BIT_ALL);
	
	rc_dev->priv = ir;
	rc_dev->open = smi_ir_open;
	rc_dev->close = smi_ir_close;
	rc_dev->timeout =  MS_TO_NS(10); */

	ir->rc_dev = rc_dev;
	ir->dev = dev;
	
	/*tasklet_init(&ir->tasklet, smi_ir_decode, (unsigned long)ir);
	tasklet_disable(&ir->tasklet); */
	INIT_WORK(&ir->work, smi_ir_decode);
	smi_ir_disableInterrupt(ir);

	ret = rc_register_device(rc_dev);
	if(ret)
		goto ir_err;	
	
	/* smi_ir_start(ir);*/
	return 0;
ir_err:
	rc_free_device(rc_dev);
	return ret;
}

static void smi_ir_exit(struct smi_dev *dev)
{
	struct smi_rc *ir = &dev->ir;	
	struct rc_dev *rc_dev = ir->rc_dev;
		
	dprintk("%s\n", __func__);
	smi_ir_stop(ir);
	/* tasklet_kill(&ir->tasklet);*/
	rc_unregister_device(rc_dev);
	ir->rc_dev = NULL;
}

/* i2c bit bus.*/
static void smi_i2c_cfg(struct smi_dev *dev, u32 sw_ctl)
{
	u32 dwCtrl;
	
	dwCtrl = smi_read(sw_ctl);
	dwCtrl &= ~0x18; /* disable output.*/
	dwCtrl |= 0x21; /* reset and software mode.*/
	dwCtrl &= ~0xff00;
	dwCtrl |= 0x6400;
	smi_write(sw_ctl, dwCtrl);
	msleep(1);
	dwCtrl = smi_read(sw_ctl);
	dwCtrl &= ~0x20;
	smi_write(sw_ctl, dwCtrl);
}
static void smi_i2c_setsda(struct smi_dev *dev, int state, u32 sw_ctl)
{
	if(state) {
		/* set as input.*/
		smi_clear(sw_ctl, SW_I2C_MSK_DAT_EN);		
	} else {
		smi_clear(sw_ctl, SW_I2C_MSK_DAT_OUT);
		/* set as output.*/
		smi_set(sw_ctl, SW_I2C_MSK_DAT_EN);
	}	
}
static void smi_i2c_setscl(void *data, int state, u32 sw_ctl)
{
	struct smi_dev *dev = data;
	if(state) {
		/* set as input.*/
		smi_clear(sw_ctl, SW_I2C_MSK_CLK_EN);		
	} else {
		smi_clear(sw_ctl, SW_I2C_MSK_CLK_OUT);
		/* set as output.*/
		smi_set(sw_ctl, SW_I2C_MSK_CLK_EN);
	}	
}
static int smi_i2c_getsda(void *data, u32 sw_ctl)
{
	struct smi_dev *dev = data;
	/* set as input.*/
	smi_clear(sw_ctl, SW_I2C_MSK_DAT_EN);
	udelay(1);		
	return (smi_read(sw_ctl) & SW_I2C_MSK_DAT_IN) ? 1 : 0;
}
static int smi_i2c_getscl(void *data, u32 sw_ctl)
{
	struct smi_dev *dev = data;
	/* set as input.*/
	smi_clear(sw_ctl, SW_I2C_MSK_CLK_EN);
	udelay(1);		
	return (smi_read(sw_ctl) & SW_I2C_MSK_CLK_IN) ? 1 : 0;
}
/* i2c 0.*/
static void smi_i2c0_setsda(void *data, int state)
{
	struct smi_dev *dev = data;
	smi_i2c_setsda(dev, state, I2C_A_SW_CTL);
}
static void smi_i2c0_setscl(void *data, int state)
{
	struct smi_dev *dev = data;
	smi_i2c_setscl(dev, state, I2C_A_SW_CTL);	
}
static int smi_i2c0_getsda(void *data)
{
	struct smi_dev *dev = data;
	return 	smi_i2c_getsda(dev, I2C_A_SW_CTL);
}
static int smi_i2c0_getscl(void *data)
{
	struct smi_dev *dev = data;
	return 	smi_i2c_getscl(dev, I2C_A_SW_CTL);
}
/* i2c 1.*/
static void smi_i2c1_setsda(void *data, int state)
{
	struct smi_dev *dev = data;
	smi_i2c_setsda(dev, state, I2C_B_SW_CTL);
}
static void smi_i2c1_setscl(void *data, int state)
{
	struct smi_dev *dev = data;
	smi_i2c_setscl(dev, state, I2C_B_SW_CTL);	
}
static int smi_i2c1_getsda(void *data)
{
	struct smi_dev *dev = data;
	return 	smi_i2c_getsda(dev, I2C_B_SW_CTL);
}
static int smi_i2c1_getscl(void *data)
{
	struct smi_dev *dev = data;
	return 	smi_i2c_getscl(dev, I2C_B_SW_CTL);
}

static int smi_i2c_init(struct smi_dev *dev)
{
	int ret;
	dprintk("%s\n", __func__);

	/* i2c bus 0 */
	smi_i2c_cfg(dev, I2C_A_SW_CTL);
	i2c_set_adapdata(&dev->i2c_bus[0], dev);
	strcpy(dev->i2c_bus[0].name, "SMI-I2C0");
	dev->i2c_bus[0].owner = THIS_MODULE;
	dev->i2c_bus[0].dev.parent = &dev->pci_dev->dev;
	dev->i2c_bus[0].algo_data = &dev->i2c_bit[0];
	dev->i2c_bit[0].data = dev;
	dev->i2c_bit[0].setsda = smi_i2c0_setsda;
	dev->i2c_bit[0].setscl = smi_i2c0_setscl;
	dev->i2c_bit[0].getsda = smi_i2c0_getsda;
	dev->i2c_bit[0].getscl = smi_i2c0_getscl;
	dev->i2c_bit[0].udelay = 12;
	dev->i2c_bit[0].timeout = 10;
	/* Raise SCL and SDA */
	smi_i2c0_setsda(dev, 1);
	smi_i2c0_setscl(dev, 1);

	ret = i2c_bit_add_bus(&dev->i2c_bus[0]);
	if (ret < 0)
		return ret;	

	/* i2c bus 1 */
	smi_i2c_cfg(dev, I2C_B_SW_CTL);
	i2c_set_adapdata(&dev->i2c_bus[1], dev);
	strcpy(dev->i2c_bus[1].name, "SMI-I2C1");
	dev->i2c_bus[1].owner = THIS_MODULE;
	dev->i2c_bus[1].dev.parent = &dev->pci_dev->dev;
	dev->i2c_bus[1].algo_data = &dev->i2c_bit[1];
	dev->i2c_bit[1].data = dev;
	dev->i2c_bit[1].setsda = smi_i2c1_setsda;
	dev->i2c_bit[1].setscl = smi_i2c1_setscl;
	dev->i2c_bit[1].getsda = smi_i2c1_getsda;
	dev->i2c_bit[1].getscl = smi_i2c1_getscl;
	dev->i2c_bit[1].udelay = 12;
	dev->i2c_bit[1].timeout = 10;
	/* Raise SCL and SDA */
	smi_i2c1_setsda(dev, 1);
	smi_i2c1_setscl(dev, 1);

	ret = i2c_bit_add_bus(&dev->i2c_bus[1]);
	if (ret < 0)
		i2c_del_adapter(&dev->i2c_bus[0]);
	
	return ret;	
}
static void smi_i2c_exit(struct smi_dev *dev)
{
	dprintk("%s\n", __func__);
	i2c_del_adapter(&dev->i2c_bus[0]);
	i2c_del_adapter(&dev->i2c_bus[1]);
}

static int smi_read_eeprom(struct i2c_adapter *i2c, u16 reg, u8 *data, u16 size)
{
	int ret;
	u8 b0[2] = { (reg >> 8) & 0xff, reg & 0xff };

	struct i2c_msg msg[] = {
		{ .addr = 0x50, .flags = 0,
			.buf = b0, .len = 2 },
		{ .addr = 0x50, .flags = I2C_M_RD,
			.buf = data, .len = size }
	};
	
	ret = i2c_transfer(i2c, msg, 2);

	if (ret != 2) {
		printk(KERN_ERR "%s: reg=0x%x (error=%d)\n",
			__func__, reg, ret);
		return ret;
	}
	return ret;
}

static void smi_port_disableInterrupt(struct smi_port *port)
{
	struct smi_dev *dev = port->dev;
	smi_write(MSI_INT_ENA_CLR, (port->_dmaInterruptCH0 | port->_dmaInterruptCH1));
}
static void smi_port_enableInterrupt(struct smi_port *port)
{
	struct smi_dev *dev = port->dev;
	smi_write(MSI_INT_ENA_SET, (port->_dmaInterruptCH0 | port->_dmaInterruptCH1));
}
static void smi_port_clearInterrupt(struct smi_port *port)
{
	struct smi_dev *dev = port->dev;
	smi_write(MSI_INT_STATUS_CLR, (port->_dmaInterruptCH0 | port->_dmaInterruptCH1));
}
/* tasklet handler: DMA data to dmx.*/
static void smi_dma_xfer(unsigned long data)
{
	struct smi_port *port = (struct smi_port *) data;
	struct smi_dev *dev = port->dev;
	u32 intr_status, finishedData, dmaManagement;
	u8 dmaChan0State, dmaChan1State;
	
	intr_status = port->_int_status;	
	dmaManagement = smi_read(port->DMA_MANAGEMENT);
	dmaChan0State = (u8)((dmaManagement & 0x00000030) >> 4);
	dmaChan1State = (u8)((dmaManagement & 0x00300000) >> 20);
	
	if(dmaChan0State == 0x10) {
		dprintk("DMA CH0 engine cancel!\n");
	} else if(dmaChan0State == 0x11) {
		dprintk("DMA CH0 engine timeout!\n");
	}
	
	if(dmaChan1State == 0x10) {
		dprintk("DMA CH1 engine cancel!\n");
	} else if(dmaChan1State == 0x11) {
		dprintk("DMA CH1 engine timeout!\n");
	}
	
	/* CH-0 DMA interrupt.*/
	if((intr_status & port->_dmaInterruptCH0) && (dmaChan0State == 0x01)) {    	
    		dprintk("Port[%d]-DMA CH0 engine complete successful !\n", port->idx);
    		finishedData = smi_read(port->DMA_CHAN0_TRANS_STATE);
    		finishedData &= 0x003FFFFF;
        	/* value of DMA_PORT0_CHAN0_TRANS_STATE register [21:0] indicate dma total transfer length
        	and zero of [21:0] indicate dma total transfer length equal to 0x400000 (4MB)*/
    		if(finishedData == 0)
    			finishedData = 0x00400000;
    		if(finishedData != SMI_TS_DMA_BUF_SIZE) {
    			dprintk("DMA CH0 engine complete length mismatched, finish data=%d !\n", finishedData);
    		}
    		dprintk("DMA CH0 data[0]=%x,data[1]=%x.\n", port->cpu_addr[0][0],port->cpu_addr[0][1]);
    		dvb_dmx_swfilter_packets(&port->demux, port->cpu_addr[0], (finishedData / 188));
    		/*dvb_dmx_swfilter(&port->demux, port->cpu_addr[0], finishedData);*/
    	}
    	/* CH-1 DMA interrupt.*/
    	if((intr_status & port->_dmaInterruptCH1) && (dmaChan1State == 0x01)) {    	
    		dprintk("Port[%d]-DMA CH1 engine complete successful !\n", port->idx);
    		finishedData = smi_read(port->DMA_CHAN1_TRANS_STATE);
    		finishedData &= 0x003FFFFF;
        	/* value of DMA_PORT0_CHAN0_TRANS_STATE register [21:0] indicate dma total transfer length
        	and zero of [21:0] indicate dma total transfer length equal to 0x400000 (4MB)*/
    		if(finishedData == 0)
    			finishedData = 0x00400000;
    		if(finishedData != SMI_TS_DMA_BUF_SIZE) {
    			dprintk("DMA CH1 engine complete length mismatched, finish data=%d !\n", finishedData);
    		}
    		dvb_dmx_swfilter_packets(&port->demux, port->cpu_addr[1], (finishedData / 188));
    		/*dvb_dmx_swfilter(&port->demux, port->cpu_addr[1], finishedData);*/
    	}	
    	/* restart DMA.*/
    	if(intr_status & port->_dmaInterruptCH0)
    		dmaManagement |= 0x00000002;
    	if(intr_status & port->_dmaInterruptCH1)
    		dmaManagement |= 0x00020000;
    	smi_write(port->DMA_MANAGEMENT, dmaManagement);      
    	/* Re-enable interrupts */
    	smi_port_enableInterrupt(port);	
}

static void smi_port_dma_free(struct smi_port *port)
{
	if(port->cpu_addr[0]) {
		pci_free_consistent(port->dev->pci_dev, SMI_TS_DMA_BUF_SIZE,
				    port->cpu_addr[0], port->dma_addr[0]);
		port->cpu_addr[0] = NULL;		    
	}
	if(port->cpu_addr[1]) {
		pci_free_consistent(port->dev->pci_dev, SMI_TS_DMA_BUF_SIZE,
				    port->cpu_addr[1], port->dma_addr[1]);
		port->cpu_addr[1] = NULL;		    
	}
}

static int smi_port_init(struct smi_port *port, int dmaChanUsed)
{
	dprintk("%s, port %d, dmaused %d\n", __func__, port->idx, dmaChanUsed);
	port->enable = 0;
	if(port->idx == 0) {
		/* Port A */
		port->_dmaInterruptCH0 = dmaChanUsed & 0x01;
		port->_dmaInterruptCH1 = dmaChanUsed & 0x02;
		
		port->DMA_CHAN0_ADDR_LOW	= DMA_PORTA_CHAN0_ADDR_LOW;
		port->DMA_CHAN0_ADDR_HI		= DMA_PORTA_CHAN0_ADDR_HI;
		port->DMA_CHAN0_TRANS_STATE	= DMA_PORTA_CHAN0_TRANS_STATE;
		port->DMA_CHAN0_CONTROL		= DMA_PORTA_CHAN0_CONTROL;
		port->DMA_CHAN1_ADDR_LOW	= DMA_PORTA_CHAN1_ADDR_LOW;
		port->DMA_CHAN1_ADDR_HI		= DMA_PORTA_CHAN1_ADDR_HI;
		port->DMA_CHAN1_TRANS_STATE	= DMA_PORTA_CHAN1_TRANS_STATE;
		port->DMA_CHAN1_CONTROL		= DMA_PORTA_CHAN1_CONTROL;
		port->DMA_MANAGEMENT		= DMA_PORTA_MANAGEMENT;
	} else {
		/* Port B */
		port->_dmaInterruptCH0 = (dmaChanUsed << 2) & 0x04;
		port->_dmaInterruptCH1 = (dmaChanUsed << 2) & 0x08;
		
		port->DMA_CHAN0_ADDR_LOW	= DMA_PORTB_CHAN0_ADDR_LOW;
		port->DMA_CHAN0_ADDR_HI		= DMA_PORTB_CHAN0_ADDR_HI;
		port->DMA_CHAN0_TRANS_STATE	= DMA_PORTB_CHAN0_TRANS_STATE;
		port->DMA_CHAN0_CONTROL		= DMA_PORTB_CHAN0_CONTROL;
		port->DMA_CHAN1_ADDR_LOW	= DMA_PORTB_CHAN1_ADDR_LOW;
		port->DMA_CHAN1_ADDR_HI		= DMA_PORTB_CHAN1_ADDR_HI;
		port->DMA_CHAN1_TRANS_STATE	= DMA_PORTB_CHAN1_TRANS_STATE;
		port->DMA_CHAN1_CONTROL		= DMA_PORTB_CHAN1_CONTROL;
		port->DMA_MANAGEMENT		= DMA_PORTB_MANAGEMENT;		
	}
	
	if(port->_dmaInterruptCH0) {
		port->cpu_addr[0] = pci_alloc_consistent(port->dev->pci_dev,
					SMI_TS_DMA_BUF_SIZE,
					&port->dma_addr[0]);
		if(!port->cpu_addr[0]) {
			dprintk("%s(): Port[%d] DMA CH0 memory allocation failed!\n", __func__, port->idx);
			goto err;
		}			
	}
			
	if(port->_dmaInterruptCH1) {
		port->cpu_addr[1] = pci_alloc_consistent(port->dev->pci_dev,
					SMI_TS_DMA_BUF_SIZE,
					&port->dma_addr[1]);			
		if(!port->cpu_addr[1]) {
			dprintk("%s(): Port[%d] DMA CH1 memory allocation failed!\n", __func__, port->idx);
			goto err;
		}
	}					
	
	smi_port_disableInterrupt(port);
	tasklet_init(&port->tasklet, smi_dma_xfer, (unsigned long)port);
	tasklet_disable(&port->tasklet);
	port->enable = 1;
	return 0;
err:
	smi_port_dma_free(port);
	return -ENOMEM;
}

static void smi_port_exit(struct smi_port *port)
{
	dprintk("%s\n", __func__);
	smi_port_disableInterrupt(port);
	tasklet_kill(&port->tasklet);
	smi_port_dma_free(port);
	port->enable = 0;
}

static void smi_port_irq(struct smi_port *port, u32 int_status)
{
	u32 port_req_irq = port->_dmaInterruptCH0 | port->_dmaInterruptCH1;
	if(int_status & port_req_irq) {
		smi_port_disableInterrupt(port);
		port->_int_status = int_status;
		smi_port_clearInterrupt(port);
		tasklet_schedule(&port->tasklet);
	}
}
static irqreturn_t smi_irq_handler(int irq, void *dev_id)
{
	struct smi_dev *dev = dev_id;
	struct smi_port *port0 = &dev->ts_port[0];
	struct smi_port *port1 = &dev->ts_port[1];
	struct smi_rc *ir = &dev->ir;
	
	u32 intr_status = smi_read(MSI_INT_STATUS);
	
	/* ts0 interrupt.*/
	if(dev->info->ts_0)
		smi_port_irq(port0, intr_status);
	
	/* ts1 interrupt.*/
	if(dev->info->ts_1)
		smi_port_irq(port1, intr_status);
		
	/* ir interrupt.*/
	smi_ir_irq(ir, intr_status);
		
	return IRQ_HANDLED;
}

static struct m88rs6000_config smi_dvbsky_s952_cfg = {
	.demod_address = 0x69, /* 0xd2 >> 1 */
	.pin_ctrl = 0x82,
	.ci_mode = 0,
	.ts_mode = 0,
	//.tuner_readstops = 1,
};

static struct m88ds3103_config smi_dvbsky_s950_cfg = {
	.demod_address = 0x68,
	.pin_ctrl = 0x82,
	.ci_mode = 0,
	.ts_mode = 0,
	//.tuner_readstops = 1,
};

static int smi_fe_init(struct smi_port *port)
{
	int ret = 0;
	struct smi_dev *dev = port->dev;
	struct dvb_adapter *adap = &port->dvb_adapter;
	struct i2c_adapter *i2c = (port->idx == 0) ? &dev->i2c_bus[0] : &dev->i2c_bus[1];
	u8 mac_ee[16];

	dprintk("%s: port %d, fe_type = %d\n", __func__, port->idx, port->fe_type);
	switch(port->fe_type) {
		case DVBSKY_FE_M88RS6000:
			port->fe = dvb_attach(m88rs6000_attach, &smi_dvbsky_s952_cfg, i2c);
		break;
		case DVBSKY_FE_M88DS3103:
			port->fe = dvb_attach(m88ds3103_attach, &smi_dvbsky_s950_cfg, i2c);		
		break;
	}
	if(!port->fe)
		return -ENODEV;
		
	ret = dvb_register_frontend(adap, port->fe);
	if(ret < 0) {
		dvb_frontend_detach(port->fe);
		return ret;
	}
	/*init MAC.*/
	ret = smi_read_eeprom(&dev->i2c_bus[0], 0xc0, mac_ee, 16);
	printk(KERN_INFO "DVBSky PCIe MAC= %pM\n", mac_ee + (port->idx)*8);
	memcpy(adap->proposed_mac, mac_ee + (port->idx)*8, 6);	
	
	return ret;
}
static void smi_fe_exit(struct smi_port *port)
{
	dprintk("%s\n", __func__);
	dvb_unregister_frontend(port->fe);
	dvb_frontend_detach(port->fe);
}
static int my_dvb_dmx_ts_card_init(struct dvb_demux *dvbdemux, char *id,
			    int (*start_feed)(struct dvb_demux_feed *),
			    int (*stop_feed)(struct dvb_demux_feed *),
			    void *priv)
{
	dvbdemux->priv = priv;

	dvbdemux->filternum = 256;
	dvbdemux->feednum = 256;
	dvbdemux->start_feed = start_feed;
	dvbdemux->stop_feed = stop_feed;
	dvbdemux->write_to_decoder = NULL;
	dvbdemux->dmx.capabilities = (DMX_TS_FILTERING |
				      DMX_SECTION_FILTERING |
				      DMX_MEMORY_BASED_FILTERING);
	return dvb_dmx_init(dvbdemux);
}

static int my_dvb_dmxdev_ts_card_init(struct dmxdev *dmxdev,
			       struct dvb_demux *dvbdemux,
			       struct dmx_frontend *hw_frontend,
			       struct dmx_frontend *mem_frontend,
			       struct dvb_adapter *dvb_adapter)
{
	int ret;

	dmxdev->filternum = 256;
	dmxdev->demux = &dvbdemux->dmx;
	dmxdev->capabilities = 0;
	ret = dvb_dmxdev_init(dmxdev, dvb_adapter);
	if (ret < 0)
		return ret;

	hw_frontend->source = DMX_FRONTEND_0;
	dvbdemux->dmx.add_frontend(&dvbdemux->dmx, hw_frontend);
	mem_frontend->source = DMX_MEMORY_FE;
	dvbdemux->dmx.add_frontend(&dvbdemux->dmx, mem_frontend);
	return dvbdemux->dmx.connect_frontend(&dvbdemux->dmx, hw_frontend);
}

static u32 smi_config_DMA(struct smi_port *port)
{	
    struct smi_dev *dev = port->dev;
    u32 totalLength = 0, dmaMemPtrLow, dmaMemPtrHi, dmaCtlReg, dmaManagement = 0;
    u8 chanLatencyTimer = 0, dmaChanEnable = 1, dmaTransStart = 1;
    u32 tlpTransUnit = DMA_TRANS_UNIT_188;
    u8 tlpTc = 0, tlpTd = 1, tlpEp = 0, tlpAttr = 0;
    u64 mem;
    
    dmaManagement = smi_read(port->DMA_MANAGEMENT);
    /* Setup Channel-0 */
    if (port->_dmaInterruptCH0) {
	totalLength = SMI_TS_DMA_BUF_SIZE;
	mem = port->dma_addr[0];	
	dmaMemPtrLow = mem & 0xffffffff;
	dmaMemPtrHi = mem >> 32;			
	dmaCtlReg = (totalLength) | (tlpTransUnit << 22) | (tlpTc << 25) |
		        (tlpTd << 28) | (tlpEp << 29) | (tlpAttr << 30);
	dmaManagement |= dmaChanEnable | (dmaTransStart << 1) | (chanLatencyTimer << 8);
	/* write DMA register, start DMA engine */
	smi_write(port->DMA_CHAN0_ADDR_LOW, dmaMemPtrLow);
	smi_write(port->DMA_CHAN0_ADDR_HI, dmaMemPtrHi);
	smi_write(port->DMA_CHAN0_CONTROL, dmaCtlReg);
    }   
    /* Setup Channel-1 */
    if (port->_dmaInterruptCH1) {
        totalLength = SMI_TS_DMA_BUF_SIZE;
	mem = port->dma_addr[1];
	dmaMemPtrLow = mem & 0xffffffff;
	dmaMemPtrHi = mem >> 32;			
	dmaCtlReg = (totalLength) | (tlpTransUnit << 22) | (tlpTc << 25) |
		        (tlpTd << 28) | (tlpEp << 29) | (tlpAttr << 30);
	dmaManagement |= (dmaChanEnable << 16) | (dmaTransStart << 17) | (chanLatencyTimer << 24);
	/* write DMA register, start DMA engine */
	smi_write(port->DMA_CHAN1_ADDR_LOW, dmaMemPtrLow);
	smi_write(port->DMA_CHAN1_ADDR_HI, dmaMemPtrHi);
	smi_write(port->DMA_CHAN1_CONTROL, dmaCtlReg);    
    }       
    return dmaManagement;
}

static int smi_start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct smi_port *port = dvbdmx->priv;
	struct smi_dev *dev = port->dev;
	u32 dmaManagement;

	if (port->users++ == 0) {
		dmaManagement = smi_config_DMA(port);
		smi_port_clearInterrupt(port);
		smi_port_enableInterrupt(port);
		smi_write(port->DMA_MANAGEMENT, dmaManagement);
		tasklet_enable(&port->tasklet);		
	}
	return port->users;
}

static int smi_stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct smi_port *port = dvbdmx->priv;
	struct smi_dev *dev = port->dev;

	if (--port->users)
		return port->users;

	tasklet_disable(&port->tasklet);
	smi_port_disableInterrupt(port);
	smi_clear(port->DMA_MANAGEMENT, 0x30003);	
	return 0;
}

static int smi_dvb_init(struct smi_port *port)
{
	int ret;
	struct dvb_adapter *adap = &port->dvb_adapter;
	struct dvb_demux *dvbdemux = &port->demux;

	dprintk("%s, port %d\n", __func__, port->idx);
	
	ret = dvb_register_adapter(adap, "SMI_DVB", THIS_MODULE,
				   &port->dev->pci_dev->dev,
				   adapter_nr);		
	if (ret < 0) {
		printk(KERN_ERR "smi_dvb: Could not register adapter."
		       "Check if you enabled enough adapters in dvb-core!\n");
		return ret;
	}	
	ret = my_dvb_dmx_ts_card_init(dvbdemux, "SW demux",
				      smi_start_feed,
				      smi_stop_feed, port);
	if (ret < 0)
		goto err_del_dvb_register_adapter;
				
	ret = my_dvb_dmxdev_ts_card_init(&port->dmxdev, &port->demux,
					 &port->hw_frontend,
					 &port->mem_frontend, adap);
	if (ret < 0)
		goto err_del_dvb_dmx;
		
	ret = dvb_net_init(adap, &port->dvbnet, port->dmxdev.demux);
	if (ret < 0)
		goto err_del_dvb_dmxdev;
	return 0;	
err_del_dvb_dmxdev:
	dvbdemux->dmx.close(&dvbdemux->dmx);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, &port->hw_frontend);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, &port->mem_frontend);
	dvb_dmxdev_release(&port->dmxdev);
err_del_dvb_dmx:
	dvb_dmx_release(&port->demux);	
err_del_dvb_register_adapter:
	dvb_unregister_adapter(&port->dvb_adapter);		
	return ret;
}

static void smi_dvb_exit(struct smi_port *port)
{
	struct dvb_demux *dvbdemux = &port->demux;
	
	dprintk("%s\n", __func__);
	
	dvb_net_release(&port->dvbnet);
	
	dvbdemux->dmx.close(&dvbdemux->dmx);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, &port->hw_frontend);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, &port->mem_frontend);
	dvb_dmxdev_release(&port->dmxdev);
	dvb_dmx_release(&port->demux);
	
	dvb_unregister_adapter(&port->dvb_adapter);
}
static int smi_port_attach(struct smi_dev *dev, struct smi_port *port, int index)
{			
	int ret, dmachs;
	dprintk("%s port %d\n", __func__, index);
	port->dev = dev;
	port->idx = index;
	
	port->fe_type = (index == 0) ? dev->info->fe_0 : dev->info->fe_1;
	dmachs = (index == 0) ? dev->info->ts_0 : dev->info->ts_1;
	
	/* port init.*/
	ret = smi_port_init(port, dmachs);
	if(ret < 0)
		return ret;
	/* dvb init.*/
	ret = smi_dvb_init(port);
	if(ret < 0)
		goto err_del_port_init;	
	/* fe init.*/
	ret = smi_fe_init(port);
	if(ret < 0)
		goto err_del_dvb_init;
	return 0;
err_del_dvb_init:
	smi_dvb_exit(port);
err_del_port_init:
	smi_port_exit(port);
	return ret;
}
static void smi_port_detach(struct smi_port *port)
{
	dprintk("%s\n", __func__);
	smi_fe_exit(port);
	smi_dvb_exit(port);
	smi_port_exit(port);
}

static int smi_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct smi_dev *dev;
	int ret = -ENOMEM;

	if (pci_enable_device(pdev) < 0)
		return -ENODEV;
		
	dev = kzalloc(sizeof(struct smi_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_pci_disable_device;
	}
		
	dev->pci_dev = pdev;
	pci_set_drvdata(pdev, dev);
	dev->info = (struct smi_cfg_info *) id->driver_data;
	printk(KERN_INFO "SMI PCIe driver detected: %s\n", dev->info->name);
	
	dev->nr = dev->info->type;
	dev->lmmio = ioremap(pci_resource_start(dev->pci_dev, 0),
			    pci_resource_len(dev->pci_dev, 0));
	if (!dev->lmmio) {
		ret = -ENOMEM;
		goto err_kfree;
	}
	
	/* should we set to 32bit DMA???*/
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret < 0)
		goto err_pci_iounmap;

	pci_set_master(pdev);
	
	ret = smi_hw_init(dev);
	if (ret < 0)
		goto err_pci_iounmap;
		
	ret = smi_i2c_init(dev);
	if (ret < 0)
		goto err_pci_iounmap;
		
	if(dev->info->ts_0) {
		ret = smi_port_attach(dev, &dev->ts_port[0], 0);
		if(ret < 0)
			goto err_del_i2c_adaptor;
	}
	
	if(dev->info->ts_1) {
		ret = smi_port_attach(dev, &dev->ts_port[1], 1);
		if(ret < 0)			
			goto err_del_port0_attach;
	}
	
	/* to do: ir init ???.*/
	ret = smi_ir_init(dev);
	if(ret < 0)
		goto err_del_port1_attach;
		
#ifdef CONFIG_PCI_MSI /* to do msi interrupt.???*/
	if (pci_msi_enabled())
		ret = pci_enable_msi(dev->pci_dev);
	if (ret) {
		dprintk("MSI not available.\n");
	}
#endif

	ret = request_irq(dev->pci_dev->irq, smi_irq_handler,
			   IRQF_SHARED, "SMI_PCIE", dev);
	if (ret < 0)
		goto err_del_ir;
		
	smi_ir_start(&dev->ir);
	return 0;
err_del_ir:
	smi_ir_exit(dev);	
err_del_port1_attach:
	if(dev->info->ts_1)
		smi_port_detach(&dev->ts_port[1]);	
err_del_port0_attach:
	if(dev->info->ts_0)
		smi_port_detach(&dev->ts_port[0]);
err_del_i2c_adaptor:	
	smi_i2c_exit(dev);
err_pci_iounmap:
	iounmap(dev->lmmio);
err_kfree:
	pci_set_drvdata(pdev, 0);
	kfree(dev);
err_pci_disable_device:
	pci_disable_device(pdev);
	return ret;
}

static void smi_remove(struct pci_dev *pdev)
{
	struct smi_dev *dev = pci_get_drvdata(pdev);
	smi_write(MSI_INT_ENA_CLR, ALL_INT);
	free_irq(dev->pci_dev->irq, dev);
#ifdef CONFIG_PCI_MSI
	pci_disable_msi(dev->pci_dev);
#endif	
	if(dev->info->ts_1)
		smi_port_detach(&dev->ts_port[1]);	
	if(dev->info->ts_0)
		smi_port_detach(&dev->ts_port[0]);
	
	smi_i2c_exit(dev);
	smi_ir_exit(dev);
	iounmap(dev->lmmio);
	pci_set_drvdata(pdev, 0);
	pci_disable_device(pdev);
	kfree(dev);
}

static struct smi_cfg_info dvbsky_s952_cfg = {
	.type = SMI_DVBSKY_S952,
	.name = "DVBSky S952 V3",
	.ts_0 = 0x03,
	.ts_1 = 0x03,
	.fe_0 = DVBSKY_FE_M88RS6000,
	.fe_1 = DVBSKY_FE_M88RS6000,
};

static struct smi_cfg_info dvbsky_s950_cfg = {
	.type = SMI_DVBSKY_S950,
	.name = "DVBSky S950 V3",
	.ts_0 = 0x00,
	.ts_1 = 0x03,
	.fe_0 = DVBSKY_FE_NULL,
	.fe_1 = DVBSKY_FE_M88DS3103,	
};

/* PCI IDs */
#define SMI_ID(_subvend, _subdev, _driverdata) {	\
	.vendor      = SMI_VID,    .device    = SMI_PID, \
	.subvendor   = _subvend, .subdevice = _subdev, \
	.driver_data = (unsigned long)&_driverdata }
	
static const struct pci_device_id smi_id_table[] = {
	SMI_ID(0x4254, 0x0552, dvbsky_s952_cfg),
	SMI_ID(0x4254, 0x0550, dvbsky_s950_cfg),
	{0}
};
MODULE_DEVICE_TABLE(pci, smi_id_table);

static struct pci_driver smipcie_driver = {
	.name = "DVBSky SMI PCIe driver",
	.id_table = smi_id_table,
	.probe = smi_probe,
	.remove = smi_remove,
};

module_pci_driver(smipcie_driver);

MODULE_DESCRIPTION("SMI PCIe driver");
MODULE_AUTHOR("Max nibble");
MODULE_LICENSE("GPL");
