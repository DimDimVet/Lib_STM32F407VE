#include "USB_FS_STM32F407.h"

/*-----------------------------------------------------------------------------------------------*/
uint32_t device_state = DEVICE_STATE_DEFAULT; /*Статус USB регистров*/
EndPointStruct EndPoint[EP_COUNT];	/*Все EP включены в этот массив.*/

/*-----------------------------------------------------------------------------------------------*/
/*Инициализация USB-port, PA11/USB_DM и PA12/USB_DP*/

void USB_Init_GPIO()
{
	AHB1_ENABLE_PERIPHERY(RCC_AHB1ENR_GPIOAEN);/*USB_DM PA11,USB_DP PA12*/
	
	GPIO_Structure USB_DM = {.GPIOx = GPIOA,.Pin = PIN11,
														.Mode = GPIO_MODE_AF,.Speed = GPIO_SPEED_VERY_HIGH,
														.Pull = GPIO_PUPDR_NOPULL,.Otyper = GPIO_OTYPER_PUSHPULL,
														.Alternate = GPIO_AF10};
	
	GPIO_Init(&USB_DM);
	
	GPIO_Structure USB_DP = {.GPIOx = GPIOA,.Pin = PIN12,
														.Mode = GPIO_MODE_AF,.Speed = GPIO_SPEED_VERY_HIGH,
														.Pull = GPIO_PUPDR_NOPULL,.Otyper = GPIO_OTYPER_PUSHPULL,
														.Alternate = GPIO_AF10};
	
	GPIO_Init(&USB_DP);

}
/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Инициализация USB-регистров*/
void USB_Init_Reg()
{
	device_state = DEVICE_STATE_DEFAULT;
	
	Write_REG(USB_OTG_FS->GAHBCFG,USB_OTG_GAHBCFG_GINT);/*Разрешим глобальное прерывание регистра USB*/
	Enable_BIT(USB_OTG_FS->GINTMSK,USB_OTG_GINTMSK_USBRST);/*Сброс USB*/
	Enable_BIT(USB_OTG_FS->GINTMSK,USB_OTG_GINTMSK_SOFM);/*Старт передачи фрейма*/
	Enable_BIT(USB_OTG_FS->GINTMSK,USB_OTG_GINTMSK_OEPINT);/*Прерывать OUT endpoints*/
	Enable_BIT(USB_OTG_FS->GINTMSK,USB_OTG_GINTMSK_IEPINT);/*Прерывать IN endpoints*/
	Enable_BIT(USB_OTG_FS->GINTMSK,USB_OTG_GINTSTS_RXFLVL);/*RxFIFO пустой*/
	
	Write_REG(USB_OTG_FS->GCCFG,USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_NOVBUSSENS);/*Отключение питания и Отключения датчика VBUS*/
	Write_REG(USB_OTG_DEVICE->DCTL,USB_OTG_DCTL_SDIS);/*Мягкое отключение*/
	Write_REG(USB_OTG_FS->GUSBCFG,USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL);/*Принудительный периферийный режим и Выбор высокоскоростного последовательного приемопередатчика USB 2.0 ULPI PHY или полноскоростного последовательного приемопередатчика USB 1.1*/
	
	//Disable_BIT(USB_OTG_FS->GUSBCFG,(0x0 << 10));/*Время выполнения USB-запроса (согласно AHB и ReferenceManual)*/
	Enable_BIT(USB_OTG_FS->GUSBCFG,(0x6 << 10));/*Время выполнения USB-запроса (согласно AHB и ReferenceManual)*/
	
	Set_FIFO_EP();
	
	/*Инициализация 1пакета(3*8 байт) EP0*/
	Clear_REG(USB_EP_OUT(0)->DOEPTSIZ);
	Enable_BIT(USB_EP_OUT(0)->DOEPTSIZ,(USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19)));/*Счетчик пакета: Это поле уменьшается до нуля после записи пакета в RxFIFO.*/
	Enable_BIT(USB_EP_OUT(0)->DOEPTSIZ,USB_CDC_MAX_PACKET_SIZE);/*Установить в дескрипторе*/
	Enable_BIT(USB_EP_OUT(0)->DOEPTSIZ,USB_OTG_DOEPTSIZ_STUPCNT);/*EP может принять 3 пакета. RM говорит установить STUPCNT = 3*/
	Enable_BIT(USB_EP_OUT(0)->DOEPTSIZ,USB_OTG_DOEPCTL_CNAK);/*Очистить NAK*/
	Enable_BIT(USB_EP_OUT(0)->DOEPTSIZ,USB_OTG_DOEPCTL_EPENA);/*Включить EP0*/
	
	Enable_BIT(USB_OTG_DEVICE->DCFG,USB_OTG_DCFG_DSPD_Msk);/*Скорость USB - FS */
	Write_REG(USB_OTG_FS->GINTSTS,(0xFFFFFFFF));/*Сбросить статус глобального прерывания*/
	Disable_BIT(USB_OTG_DEVICE->DCTL,USB_OTG_DCTL_SDIS);/*Отключить - Мягкое отключение*/
	
	Init_EP();
	
	NVIC_SetPriority(OTG_FS_IRQn, 6);
	NVIC_EnableIRQ(OTG_FS_IRQn);
	
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Инициализация EndPoints*/
uint8_t rx_Buffer_Ep0[RX_BUFFER_EP0_SIZE]; /*Полученные данные сохраняются здесь после того, как приложение считывает FIFO. Приёмный FIFO является общим*/
uint8_t rx_Buffer_Ep1[RX_BUFFER_EP1_SIZE]; /*Полученные данные сохраняются здесь после того, как приложение считывает FIFO. Приёмный FIFO является общим*/

void Init_EP()
{
	for(uint32_t i = 0; i < EP_COUNT; i++)
	{
		EndPoint[i].status_Rx = EP_READY;
		EndPoint[i].status_Tx = EP_READY;
		EndPoint[i].rxCounter = 0;
		EndPoint[i].tx_Counter = 0;
		EndPoint[i].Set_Tx_Buffer = &USB_Set_Tx_Buffer;
		EndPoint[i].tx_Call_Back = &USB_Transfer_TX_Callback;
				
		/*EndPoint 0*/
		if(i==0)
		{
			EndPoint[i].rxBuffer_ptr = rx_Buffer_Ep0;/*Буфер RX для EP0*/
			EndPoint[i].rxCallBack = &USB_transfer_RX_Callback_EP0;
		}
		/* EndPoint 1 */
		else if(i==1)
		{
			EndPoint[i].rxBuffer_ptr = rx_Buffer_Ep1;/*Буфер RX для EP1*/
			EndPoint[i].rxCallBack = &USB_Transfer_RX_Callback_EP1;
		}
		else
		{
			EndPoint[i].rxBuffer_ptr = 0; /*добавить поддержку EP2*/
		}
	}	
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Установка Tx буфера*/
uint32_t USB_Set_Tx_Buffer(uint8_t EPnum, uint8_t *tx_Buff, uint16_t len)
{
	/****************** Предыдущая транзакция не завершена ****************/
	if((EndPoint[EPnum].tx_Counter != 0) || (EndPoint[EPnum].status_Tx == EP_ZLP) || (Check_Device_Status(DEVICE_STATE_TX_PR) == EP_OK))
	{				
		return EP_FAILED;
	}
	
	/************** длина превышает максимальный размер TX в настройках пользователя ****************/
	uint32_t max_transfer_sz;
	if(EPnum == 0)
	{
		max_transfer_sz = MAX_CDC_EP0_TX_SIZ + (USB_CDC_MAX_PACKET_SIZE - 1);
	}
	else 
	{
		max_transfer_sz = MAX_CDC_EP1_TX_SIZ + (USB_CDC_MAX_PACKET_SIZE - 1);
	}
	
	if(len > max_transfer_sz)
	{	
		return EP_FAILED;
	}
	
	/****************** Все условия в порядке ****************/
	if(len!=0)
	{	/*Установить данные для отправки*/	
		EndPoint[EPnum].tx_Buffer_ptr = tx_Buff;
		EndPoint[EPnum].tx_Counter = len;

		/*Отправить данные*/		
		Set_Device_Status(DEVICE_STATE_TX_PR);
		EndPoint[EPnum].tx_Call_Back(EPnum);
		
		return EP_OK;
	}
	else
	{ /*Пакет нулевой длины*/	
		
		Clear_REG(USB_EP_IN(EPnum)->DIEPTSIZ);

		Write_REG(USB_EP_IN(EPnum)->DIEPTSIZ,((USB_OTG_DIEPTSIZ_PKTCNT_Msk & (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos))));/*Один пакет*/
		Disable_BIT(USB_EP_IN(EPnum)->DIEPTSIZ,USB_OTG_DIEPTSIZ_XFRSIZ_Msk);/*Пакет нулевой длинны*/
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_CNAK);/*Чистим NAK*/
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_EPENA);/*Включить EP*/
		
		Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_CNAK);/*Чистим NAK*/
		Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_EPENA);/*Включить EP*/

		return EP_OK;
	}

}
/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Загрузить транзакцию TX для определенного EP*/
uint32_t USB_Transfer_TX_Callback(uint8_t EPnum)
{
	/****************** Если занят передачей ****************/
	if(EndPoint[EPnum].status_Tx == EP_BUSY)
	{
		Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_CNAK);/*Чистим NAK*/
		Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_EPENA);/*Включить EP*/

		return EP_FAILED;
	}
	
	/************** EP готов к работе *****************/	
	/*Нет данных в буфере передачи*/
	if(EndPoint[EPnum].tx_Counter==0)
	{
		if(EndPoint[EPnum].status_Tx == EP_ZLP)
		{			
			if(Send_ZLP(EPnum) == EP_OK)
			{
				return EP_OK;
			}
			else
			{
				return EP_FAILED;
			}
			
		}
		
		Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_CNAK);/*Чистим NAK*/
		Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_EPENA);/*Включить EP*/
	
		Clear_Device_Status(DEVICE_STATE_TX_PR);
		
		EndPoint[EPnum].status_Tx = EP_READY;
		
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_SNAK);/*Установка NAK*/
		
		return EP_OK;
	}
	
	/****************** Подсчитать количество пакетов *****************/
	uint16_t max_tx_sz; /*максимальный размер передачи с учетом размера TX FIFO*/
	
	if(EPnum == 0)
	{
		max_tx_sz = MAX_CDC_EP0_TX_SIZ;
	}
	else
	{
		max_tx_sz = MAX_CDC_EP1_TX_SIZ;
	}
	
	/*если размер FIFO составляет 64 байта, а транзакция — 67 байт (DevDescriptor)
		транзакция будет разделена на две части - первая: длиной 64 байта вторая: длиной 3 байта*/
	uint32_t len;
	uint32_t residue;
	uint32_t pct_cnt;
		
	if(EndPoint[EPnum].tx_Counter < USB_CDC_MAX_PACKET_SIZE)
	{
		len = EndPoint[EPnum].tx_Counter;
		residue = 0;
		pct_cnt = 1;
	}
	else
	{
		if(EndPoint[EPnum].tx_Counter > max_tx_sz)
		{
			len = max_tx_sz;
		}
		else
		{
			len = EndPoint[EPnum].tx_Counter;
		}

		if((len % USB_CDC_MAX_PACKET_SIZE) == 0)
		{
			residue = 0;
		}
		else
		{
			residue = 1;
		}
		
		pct_cnt = (uint32_t)((len / USB_CDC_MAX_PACKET_SIZE) + residue);
	}

	/************** Установить флаг «Занято» и начать передачу *****************/

	EndPoint[EPnum].status_Tx = EP_BUSY;	

	/************** data >= USB_CDC_MAX_PACKET_SIZE *****************/


	if(EndPoint[EPnum].tx_Counter >= USB_CDC_MAX_PACKET_SIZE)
	{  /*counter >= 64*/
		uint32_t temp_one_packet = ((USB_OTG_DIEPTSIZ_PKTCNT & (pct_cnt << USB_OTG_DIEPTSIZ_PKTCNT_Pos)));/*Один пакет.Количество пакетов*/
		uint32_t temp_max_size = ((USB_OTG_DIEPTSIZ_XFRSIZ & ((len) << USB_OTG_DIEPTSIZ_XFRSIZ_Pos)));/*Макс размер передачи*/
		
		Write_REG(USB_EP_IN(EPnum)->DIEPTSIZ,(temp_one_packet | temp_max_size));
		
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_CNAK);/*Чистим NAK*/
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_EPENA);/*Включить EP*/
		
		while(Write_Fifo(EPnum, EndPoint[EPnum].tx_Buffer_ptr, (uint16_t)len) == EP_FAILED )
		{
		
			Recovery_Routine_EP_IN(EPnum); 
			
			Clear_REG(USB_EP_IN(EPnum)->DIEPTSIZ);

			uint32_t temp_one_packet = ((USB_OTG_DIEPTSIZ_PKTCNT & (pct_cnt << USB_OTG_DIEPTSIZ_PKTCNT_Pos)));/*Один пакет.Количество пакетов*/
			uint32_t temp_max_size = ((USB_OTG_DIEPTSIZ_XFRSIZ & ((len) << USB_OTG_DIEPTSIZ_XFRSIZ_Pos)));/*Макс размер передачи*/
		
			Write_REG(USB_EP_IN(EPnum)->DIEPTSIZ,(temp_one_packet | temp_max_size));
		
			Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_CNAK);/*Чистим NAK*/
			Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_EPENA);/*Включить EP*/

			if((EPnum == 1) & Check_Free_Space_Fifo(EPnum, EP1_MIN_DTFXSTS_LVL))
			{
				USB_Flush_Tx_Fifo(1, FLUSH_FIFO_TIMEOUT);
			}
			
		}	

		USB_Flush_Tx_Fifo(1, FLUSH_FIFO_TIMEOUT);/*Сбросим fifo*/

		/*Этот пакет из MaxSize был последним в очереди, если требуется 0-пакет (ZLP)*/
		if(EndPoint[EPnum].tx_Counter==0 && residue == 0)
		{ 
			EndPoint[EPnum].status_Tx = EP_ZLP; /*изменить статус EP TX на ZLP, после этого ZLP будет отправлен в последовательном вызове функции*/
			
			while(Endpoint_Enable_Stuck(EPnum) != EP_OK)
			{
			
			}
			
			Send_ZLP(EPnum);
			
			return EP_OK;
		}
	}
	/************** data <  USB_CDC_MAX_PACKET_SIZE *****************/

	else if(EndPoint[EPnum].tx_Counter < USB_CDC_MAX_PACKET_SIZE)
	{
		
		uint32_t temp_one_packet = ((USB_OTG_DIEPTSIZ_PKTCNT & (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos)));/*Один пакет.Количество пакетов*/
		uint32_t temp_max_size = ((USB_OTG_DIEPTSIZ_XFRSIZ & ((EndPoint[EPnum].tx_Counter) << USB_OTG_DIEPTSIZ_XFRSIZ_Pos)));/*Макс размер передачи*/
		
		Write_REG(USB_EP_IN(EPnum)->DIEPTSIZ,(temp_one_packet | temp_max_size));
		
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_CNAK);/*Чистим NAK*/
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_EPENA);/*Включить EP*/
			
		while(Write_Fifo(EPnum, EndPoint[EPnum].tx_Buffer_ptr, (uint16_t)len) == EP_FAILED )
		{
			Recovery_Routine_EP_IN(EPnum); 
			
			Clear_REG(USB_EP_IN(EPnum)->DIEPTSIZ);
			
			uint32_t temp_one_packet = ((USB_OTG_DIEPTSIZ_PKTCNT & (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos)));/*Один пакет.Количество пакетов*/
			uint32_t temp_max_size = ((USB_OTG_DIEPTSIZ_XFRSIZ & ((EndPoint[EPnum].tx_Counter) << USB_OTG_DIEPTSIZ_XFRSIZ_Pos)));/*Макс размер передачи*/
		
			Write_REG(USB_EP_IN(EPnum)->DIEPTSIZ,(temp_one_packet | temp_max_size));
		
			Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_CNAK);/*Чистим NAK*/
			Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_EPENA);/*Включить EP*/
			
			if((EPnum == 1) & Check_Free_Space_Fifo(EPnum, EP1_MIN_DTFXSTS_LVL))
			{
				USB_Flush_Tx_Fifo(1, FLUSH_FIFO_TIMEOUT);
			}
		}
		
		Clear_Device_Status(DEVICE_STATE_TX_PR);		
	} 
	else
	{
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_CNAK);/*Чистим NAK*/
	}
	
	/************** завершить передачу и установить флаг готовности *****************/
	
	Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_CNAK);/*Чистим NAK*/
  Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_EPENA);/*Включить EP*/
		
	EndPoint[EPnum].status_Tx = EP_READY;
	
	Clear_Device_Status(DEVICE_STATE_TX_PR);
	
	return EP_OK;
}
/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Отправить пакет нулевой длины*/

uint32_t Send_ZLP(uint8_t EPnum)
{
	Clear_REG(USB_EP_IN(EPnum)->DIEPTSIZ);

	Write_REG(USB_EP_IN(EPnum)->DIEPTSIZ,((USB_OTG_DIEPTSIZ_PKTCNT_Msk & (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos))));/*Один пакет*/
	Disable_BIT(USB_EP_IN(EPnum)->DIEPTSIZ,USB_OTG_DIEPTSIZ_XFRSIZ_Msk);/*Пакет нулевой длинны*/
	Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_CNAK);/*Чистим NAK*/
	Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_EPENA);/*Включить EP*/

	while(USB_EP_IN(EPnum)->DIEPTSIZ != 0) /* ждем, что zlp удален */
	{
	
	}

	Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_CNAK);/*Чистим NAK*/
	Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_EPENA);/*Включить EP*/
	
	Clear_Device_Status(DEVICE_STATE_TX_PR);

	EndPoint[EPnum].status_Tx = EP_READY;
	Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_SNAK);/*Установка NAK*/

	if(Endpoint_Enable_Stuck(EPnum) == EP_OK)
	{
		return EP_OK;
	}
	else
	{
		return EP_FAILED;
	}
}
/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Включение EP зависло*/
uint32_t Endpoint_Enable_Stuck(uint8_t EPnum)
{
	if((USB_EP_IN(EPnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)  & /*EPENA зависло*/
		!(USB_EP_IN(EPnum)->DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ) &	/*нет ожидающих данных*/
		((USB_EP_IN(EPnum)->DIEPTSIZ & USB_OTG_HCTSIZ_PKTCNT) != 0))/*количество ожидающих пакетов*/
		{		
			return EP_FAILED;
		}
	else
		{
			return EP_OK;
		}	
}
/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Если EP IN занята и данные застряли в TX FIFO*/

uint32_t Recovery_Routine_EP_IN(uint8_t EPnum)
{
	if((EPnum ==1) & (Endpoint_Enable_Stuck(1) == EP_FAILED))
	{
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_EPDIS);/*Отключить EP*/
		Enable_BIT(USB_EP_IN(EPnum)->DIEPCTL,USB_OTG_DIEPCTL_SNAK);/*Установка NAK*/
		Clear_REG(USB_EP_IN(EPnum)->DIEPTSIZ);
		return EP_OK;
	}
	return EP_FAILED;
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Установить размер и смещение FIFO RX и TX для каждого EP*/
void Set_FIFO_EP()
{
	Write_REG(USB_OTG_FS->GRXFSIZ,RX_FIFO_SIZE);/*все EP RX FIFO размер RAM*/
	Write_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ,((TX_EP0_FIFO_SIZE << 16) | RX_FIFO_SIZE));/*EP0 TX FIFO размер RAM*/
	Write_REG(USB_OTG_FS->DIEPTXF[0],(((TX_EP1_FIFO_SIZE) << 16) | (RX_FIFO_SIZE+TX_EP0_FIFO_SIZE)));/*EP1 TX FIFO размер RAM*/
	
	for(uint32_t i = 1; i < 0x0F ; i++)
	{
		Clear_REG(USB_OTG_FS->DIEPTXF[i]);
	}
}
/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*проверить свободное место в Fifo*/
uint32_t Check_Free_Space_Fifo(uint8_t dfifo, uint32_t space)
{
	if(USB_EP_IN(dfifo)->DTXFSTS >= space)
	{
		return EP_OK;
	}
	else return EP_FAILED;
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*сброс TxFifo*/

uint32_t USB_Flush_Tx_Fifo(uint32_t EPnum, uint32_t timeout)
{
	uint32_t count = 0;
	
	Write_REG(USB_OTG_FS->GRSTCTL,(USB_OTG_GRSTCTL_TXFFLSH | (EPnum << 6)));
	
	do
	{
		if (++count > timeout)
		{
			return EP_FAILED;
		}
	}
	while ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

  return EP_OK;
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Записать данные в FIFO*/

uint32_t Write_Fifo(uint8_t dfifo, uint8_t *src, uint16_t len)
{ 
	uint16_t residue;
	
	if(len % 4 == 0)
	{
		residue = 0;
	}
	else
	{
		residue = 1;
	}
	
	uint32_t block_cnt = (uint32_t)((len/4) + residue);
	uint32_t dtxfsts_sample = USB_EP_IN(dfifo)->DTXFSTS;
		
	for (uint32_t i = 0; (i < block_cnt) ; i++)
	{
		USB_OTG_DFIFO(dfifo) = *((uint32_t *)(void *)src);
		src+=4;	
	} 

	if(DTFXSTS_timeout(dfifo, dtxfsts_sample)==EP_OK)
	{
		EndPoint[dfifo].tx_Buffer_ptr = EndPoint[dfifo].tx_Buffer_ptr + len;
		EndPoint[dfifo].tx_Counter = (uint16_t)(EndPoint[dfifo].tx_Counter - len);
		
		return EP_OK;	
	}
	else 
	{
		return EP_FAILED;
	} 
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Функция вызывается для обеспечения корректной записи данных в FIFO*/

uint32_t DTFXSTS_timeout(uint8_t Epnum, uint32_t dtxfsts_val)
{
	if(Epnum ==0) return EP_OK;

	uint32_t count = 0;
	
	do{
		if (++count > DTFXSTS_TIMEOUT)
		{
			return EP_FAILED;
		}
	} while (!(USB_EP_IN(Epnum)->DTXFSTS == dtxfsts_val));

	return EP_OK;
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Функции состояния устройства. Установка/очистка/проверка.*/
void Set_Device_Status(eDeviceState state)
{
	device_state |= state;
}

void Clear_Device_Status(eDeviceState state)
{
	device_state &= ~state;
}

uint32_t Check_Device_Status(eDeviceState state)
{
	if(device_state & state)
	{
		return EP_OK;
	}
	else return EP_FAILED;
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Функция с полученными данными (EP0) и обновить счетчик буфета EP*/
uint8_t lineCoding[CDC_LINE_CODING_LENGTH]=
{
	0x00, 
	0x00,	/*0x01,*/
	0x00, /*0xC2,*/
	0x00, /*0X0001C200 -= 115200 Kb/s*/
	0x00,
	0x00,
	0x00	/*0x08*/
};

uint32_t USB_transfer_RX_Callback_EP0(uint32_t param)
{
	uint16_t len = EndPoint[0].rxCounter;	
	
	if(len==0)
	{
		return EP_OK;
	}
	
	if(EndPoint[0].status_Rx == EP_BUSY)
	{
		return EP_FAILED;
	}
	
	if(param == CDC_SET_LINE_CODING)
	{
		Toggle_Rx_EP_Status(0, EP_BUSY);
		
		uint8_t *data = EndPoint[0].rxBuffer_ptr - EndPoint[0].rxCounter;
		
		EndPoint[0].rxBuffer_ptr = data;
		uint8_t new_linecoding_settings[CDC_LINE_CODING_LENGTH];  
		
		for(int i = 0; i < len; i++)
		{
			new_linecoding_settings[i] = *data++;
			EndPoint[0].rxCounter--;
		}

		for(uint32_t i = 0; i < CDC_LINE_CODING_LENGTH; i++)
		{
			if(i==6 && lineCoding[i]==0x08)
			{
				lineCoding[i] = 0x08;
			}
			else
			{
				lineCoding[i] = new_linecoding_settings[i];
			}
		}	
		
		/*Переключить статус EP RX*/
		Toggle_Rx_EP_Status(0, EP_READY);
	}
	return EP_OK;
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Изменить статус EP OUT*/

void Toggle_Rx_EP_Status(uint8_t EPnum, uint8_t param)
{
	if(EndPoint[EPnum].status_Rx == param)
	{
		return;
	}
	/*переключить статус*/
 	EndPoint[EPnum].status_Rx = param;

	if(param==EP_READY)
	{
		Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_CNAK);/*Чистим NAK*/
		Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_EPENA);/*Включить EP*/
 	}
	else
	{
		Enable_BIT(USB_EP_OUT(EPnum)->DOEPCTL,USB_OTG_DOEPCTL_SNAK);/*Установим NAK*/
	}
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Функция с полученными данными (EP1) и обновить счетчик буфета EP1*/

uint32_t USB_Transfer_RX_Callback_EP1(uint32_t param)
{
	if(EndPoint[1].status_Rx == EP_BUSY)
	{
		return EP_FAILED;
	}
		
 	uint16_t len = EndPoint[1].rxCounter;
	
	/*сбросить счетчик приема и указатель буфера*/
 	EndPoint[1].rxBuffer_ptr -= EndPoint[1].rxCounter;
	EndPoint[1].rxCounter = 0;	

	USB_Handler_Recieve_data(len);/*Обработчик на получения данных*/
	
	Clear_REG(USB_EP_OUT(1)->DOEPTSIZ);	
	
	Enable_BIT(USB_EP_OUT(1)->DOEPTSIZ,(USB_OTG_DOEPTSIZ_PKTCNT & (DOEPT_TRANSFER_PCT << USB_OTG_DOEPTSIZ_PKTCNT_Pos)));/*Счетчик пакета: Это поле уменьшается до нуля после записи пакета в RxFIFO.*/
	Enable_BIT(USB_EP_OUT(1)->DOEPTSIZ,DOEPT_TRANSFER_SIZE);
	Enable_BIT(USB_EP_OUT(1)->DOEPTSIZ,USB_OTG_DOEPCTL_CNAK);/*Очистить NAK*/
	Enable_BIT(USB_EP_OUT(1)->DOEPTSIZ,USB_OTG_DOEPCTL_EPENA);/*Включить EP1*/

	return param;
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Обработчик по прерыванию на получения данных*/

//uint32_t USB_Handler_Recieve_data(uint16_t length)
//{
//	return length;
//}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Функция передачи данных*/

uint32_t USB_send_data(uint8_t *txBuff, uint16_t len)
{
	if(len==0)
	{
		return EP_OK;
	}
	
	if(Is_Tx_Ep_Fifo_Ready(1,1) == EP_OK)
	{
		EndPoint[1].Set_Tx_Buffer(1, txBuff, len);
		return EP_OK;
	}
	else
	{
		return EP_FAILED;
	}
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Проверка готовности TX FIFO к отправке данных*/

uint32_t Is_Tx_Ep_Fifo_Ready(uint8_t EPnum, uint32_t param)
{
	if((param > 0) & !(device_state & DEVICE_STATE_TX_PR) & /*Передача TX активна*/
		!(USB_EP_IN(EPnum)->DIEPTSIZ & USB_OTG_DIEPTSIZ_XFRSIZ) & /*Размер передачи*/
		((USB_EP_IN(EPnum)->DIEPTSIZ & USB_OTG_HCTSIZ_PKTCNT) == 0) & /*Количество пакетов*/
		!(USB_EP_IN(EPnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) & /*Включена EP*/
		((USB_EP_IN(EPnum)->DIEPINT & USB_OTG_DIEPINT_TXFE) != 0)) /*Передача FIFO пуста*/
	{
		return EP_OK;
	}
	else
	{
		return EP_FAILED;
	}
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Прерывание по факту подключение к хосту*/

void OTG_FS_IRQHandler()
{
	/*Событие старта пакета*/

	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_SOF)
	{ 
		USB_transmit_scheduler();

		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_SOF);  	
		return;
	}
	
	/*Событие сброса USBRST*/

	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST)
	{  
		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_USBRST);
		
		Enumerate_Reset();
		return;
	 }
	 
	/*Событие EP IN*/
	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_IEPINT)
	{ 	
		uint32_t epnums  = USB_OTG_DEVICE->DAINT; /*Считывание битов прерывания EP*/

		/*Биты прерывания EP соответствуют EP0 IN*/
		if(epnums & 0x0001)
		{	
			uint32_t IN_interrupt = USB_EP_IN(0)->DIEPINT; /*Считывание регистра прерывания для EP0 IN*/

			if(IN_interrupt & USB_OTG_DIEPINT_XFRC)
			{ /*Прерывание завершения передачи*/ 
				EndPoint[0].tx_Call_Back(0); /*Процесс передачи TX (если буфер TX не пуст)*/
			}
			
			USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_IEPINT);	
			
			CLEAR_IN_EP_INTERRUPT(0, IN_interrupt);
		}
		
		/*Биты прерывания EP соответствуют EP1 IN*/
		if( epnums & 0x0002)
		{ 
			uint32_t IN_interrupt = USB_EP_IN(1)->DIEPINT;

			if(IN_interrupt & USB_OTG_DIEPINT_XFRC)
			{ /*Прерывание завершения передачи*/ 

				EndPoint[1].tx_Call_Back(1); /*Процесс передачи TX (если буфер TX не пуст)*/

				USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_IEPINT);	
				
				CLEAR_IN_EP_INTERRUPT(1, USB_OTG_DIEPINT_XFRC);			
			}
		}
		return;
	}
	
	/*Событие EP OUT*/
	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_OEPINT)
	{ /* OUT endpoint event */
		USB_CLEAR_INTERRUPT(USB_OTG_GINTSTS_OEPINT);	
		uint32_t epnums  = USB_OTG_DEVICE->DAINT; /*Считывание битов прерывания EP*/
		
		/*Биты прерывания EP соответствуют EP0 OUT*/
		if( epnums & 0x00010000)
		{  
			uint32_t epint = USB_EP_OUT(0)->DOEPINT; /*Считывание регистра прерывания для EP0 OUT*/
						
			if(epint & USB_OTG_DOEPINT_STUP)
			{	/*Получен установочный пакет*/ 			
				Enumerate_Setup();				
			}
			
			if(epint & USB_OTG_DOEPINT_XFRC)
			{
				EndPoint[0].rxCallBack(0);
				///* CNAK and EPENA must be set again after every interrupt to let this EP recieve upcoming data */		
				//USB_EP_OUT(0)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
				Enable_BIT(USB_EP_OUT(0)->DOEPCTL,USB_OTG_DOEPCTL_CNAK);/*Очистить NAK*/
				Enable_BIT(USB_EP_OUT(0)->DOEPCTL,USB_OTG_DOEPCTL_EPENA);/*Включить EP0*/
			}
			
			CLEAR_OUT_EP_INTERRUPT(0, epint);
		}

		/*Биты прерывания EP соответствуют EP1 OUT*/		
		if( epnums & 0x00020000)
		{ 
			uint32_t epint = USB_EP_OUT(1)->DOEPINT; /*Считывание регистра прерывания для EP1 OUT*/
			
			if(epint & USB_OTG_DOEPINT_XFRC)
			{	
				EndPoint[1].rxCallBack(EP_OK);	
				///* CNAK and EPENA must be set again after every interrupt to let this EP recieve upcoming data */
				//USB_EP_OUT(1)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
				Enable_BIT(USB_EP_OUT(1)->DOEPCTL,USB_OTG_DOEPCTL_CNAK);/*Очистить NAK*/
				Enable_BIT(USB_EP_OUT(1)->DOEPCTL,USB_OTG_DOEPCTL_EPENA);/*Включить EP0*/
			}
			
			CLEAR_OUT_EP_INTERRUPT(1, epint);		
		}   				
		return;
	}
	
	/*Событие RXFLVL*/
	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL)
	{ /*RX fifo не пустое событие*/
		uint32_t temp = USB_OTG_FS->GRXSTSP; /*Ознакомление со статусом пакета*/ 
		USB_MASK_INTERRUPT(USB_OTG_GINTSTS_RXFLVL); 
		
		/*Данные для (EP0, EP1)*/
		
		/*0010: получен пакет данных OUT*/
		if(((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_DATA_UPDT)
		{  
		
			if(temp & USB_OTG_GRXSTSP_BCNT)
			{ /*Количество байт > 0 ???*/ 
				uint16_t length = ((temp & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos);
				uint8_t EpNum =	((temp & USB_OTG_GRXSTSP_EPNUM) >> USB_OTG_GRXSTSP_EPNUM_Pos);

				Read_Fifo(EpNum, length); /*Чтение данных из FIFO*/
			}
		}
		/*0110: получен пакет данных SETUP в EP0*/
		else  if(((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_SETUP_UPDT)
		{ 
			Read_Setup_Fifo();
		} 
		USB_UNMASK_INTERRUPT(USB_OTG_GINTSTS_RXFLVL); 
		return;
  }  
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Планировщик USB-передачи*/

uint32_t USB_transmit_scheduler()
{
	return EP_FAILED; 
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Обрабатывать все запросы хоста, отправлять все данные дескрипторов*/
USB_setup_req_data setup_pkt_data;

void Enumerate_Setup()
{
	uint16_t len = setup_pkt_data.setup_pkt.wLength;
	uint8_t dest[128]={0};
			
	switch(setup_pkt_data.setup_pkt.wRequest)
	{
		
		case REQ_TYPE_HOST_TO_DEVICE_GET_DEVICE_DECRIPTOR:
			switch(setup_pkt_data.setup_pkt.wValue)
			{
				case DESCRIPTOR_TYPE_DEVICE: /*Request 0x0680 Value 0x0100 */
					if(DEVICE_DESCRIPTOR_LENGTH < len)
					{
						len = DEVICE_DESCRIPTOR_LENGTH;
					}
					
					memcpy(&dest, Get_Device_Descriptor(), len);
					break;
					
				case DESCRIPTOR_TYPE_CONFIGURATION: /*Request 0x0680 Value 0x0200 */
					if(CONFIGURATION_DESCRIPTOR_LENGTH < len)
					{
						len = CONFIGURATION_DESCRIPTOR_LENGTH;
					}
					
					memcpy(&dest, Get_Configuration_Descriptor(), len);
					break;
					
				case DESCRIPTOR_TYPE_DEVICE_QUALIFIER: /*Request 0x0680 Value 0x0600 */
					if(DEVICE_QUALIFIER_LENGTH < len)
					{
						len = DEVICE_QUALIFIER_LENGTH;
					}
					 
					memcpy(&dest, Get_Device_Qualifier_Descriptor(), len);
					break; 
					
				case DESCRIPTOR_TYPE_LANG_STRING: /*Request 0x0680 Value 0x0300 */
					if(LANG_DESCRIPTOR_LENGTH < len)
					{
						len = LANG_DESCRIPTOR_LENGTH;
					}
					   
					memcpy(&dest, Get_Language_String_Descriptor(), len);				
					break; 
					
				case DESCRIPTOR_TYPE_MFC_STRING: /*Request 0x0680  Value 0x0301 */
					if(MFC_DESCRIPTOR_LENGTH < len)
					{
						len = MFC_DESCRIPTOR_LENGTH;
					}
					
					memcpy(&dest, Get_Manufactor_String_Descriptor(), len);				
					break;
					
				case DESCRIPTOR_TYPE_PROD_STRING: /*Request 0x0680  Value 0x0302 */
					if(PRODUCT_DESCRIPTOR_LENGTH < len)
					{
						len = PRODUCT_DESCRIPTOR_LENGTH;
					}
					
					memcpy(&dest, Get_Product_String_Descriptor(), len);				
					break;  
					
				case DESCRIPTOR_TYPE_SERIAL_STRING: /*Request 0x0680  Value 0x0303 */
					if(SERIAL_DESCRIPTOR_LENGTH < len)
					{
						len = SERIAL_DESCRIPTOR_LENGTH;
					}
					
					memcpy(&dest, Get_Serial_Number_String_Descriptor(), len);
					break;
					
				case DESCRIPTOR_TYPE_CONFIGURATION_STRING: /*Request 0x0680  Value 0x0304 */
					if(CONFIG_STRING_LENGTH < len)
					{
						len = CONFIG_STRING_LENGTH;
					}
										
					memcpy(&dest, Get_Configuration_String_Descriptor(), len);
					break;
					
				case DESCRIPTOR_TYPE_INTERFACE_STRING: /*Request 0x0680  Value 0x0305 */
					if(INTERFACE_STRING_LENGTH < len)
					{
						len = INTERFACE_STRING_LENGTH;
					}
					
					memcpy(&dest, Get_String_Interface(), len);
					break;
					
				default:
					return;
			}
			break;
			
		case REQ_TYPE_DEVICE_TO_HOST_SET_ADDRESS: /*Request 0x0500*/
			Enable_BIT(USB_OTG_DEVICE->DCFG,(setup_pkt_data.setup_pkt.wValue << 4));
			//USB_OTG_DEVICE->DCFG |= (uint32_t)(setup_pkt_data.setup_pkt.wValue << 4);
			Set_Device_Status(DEVICE_STATE_ADDRESSED);
			break;
			
		case REQ_TYPE_DEVICE_TO_HOST_SET_CONFIGURATION: /*Request 0x0900  */
			len=0;
			break;     
		
		case CDC_GET_LINE_CODING: /*Request 0x21A1  */
			if(CDC_LINE_CODING_LENGTH < len)
			{
				len = CDC_LINE_CODING_LENGTH;
			}
			
			memcpy(&dest, &lineCoding, len);
			
			Set_Device_Status(DEVICE_STATE_LINECODED);
			break;
		
		case CDC_SET_LINE_CODING: /*Request 0x2021  */
			len=0;		
			EndPoint[0].rxCallBack(CDC_SET_LINE_CODING);
			break;   
			
		case CDC_SET_CONTROL_LINE_STATE: /*Request 0x2221  */
			len=0;	
			break;	
			
		case CLEAR_FEATURE_ENDP: /*Request 0x0201  */
			return;
			
		default:
			break;
	} 
	
	EndPoint[0].Set_Tx_Buffer(0, dest, len);
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Чтение данных из FIFO (в rxBufferMain[RX_BUFFER_MAIN_SIZE] для EP1*/

void Read_Fifo(uint8_t dfifo, uint16_t len)
{
	uint16_t residue = (len%4==0) ? 0 : 1 ;
	uint32_t block_cnt = (uint32_t)((len/4) + residue);
	uint8_t *tmp_ptr = EndPoint[dfifo].rxBuffer_ptr;

	/*Если длина необработанных данных превышает максимальную длину буфера, их необходимо перезаписать.*/
	if((dfifo == 1) & ((EndPoint[dfifo].rxCounter + len) > RX_BUFFER_EP1_SIZE))
	{
		EndPoint[dfifo].rxBuffer_ptr = rx_Buffer_Ep1;
		EndPoint[dfifo].rxCounter = 0;
	}
	
	for (uint32_t i = 0; i < block_cnt; i++)
	{
		*(uint32_t *)(void *)EndPoint[dfifo].rxBuffer_ptr = USB_OTG_DFIFO(0);
		EndPoint[dfifo].rxBuffer_ptr = EndPoint[dfifo].rxBuffer_ptr + 4;
	}

	if(dfifo!=0)
	{	
		USB_EP_OUT(dfifo)->DOEPTSIZ = 0;			
		USB_EP_OUT(dfifo)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (DOEPT_TRANSFER_PCT << USB_OTG_DOEPTSIZ_PKTCNT_Pos)); 
		USB_EP_OUT(dfifo)->DOEPTSIZ |= DOEPT_TRANSFER_SIZE; 
		USB_EP_OUT(dfifo)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}
	
	EndPoint[dfifo].rxBuffer_ptr = tmp_ptr + len;
	EndPoint[dfifo].rxCounter = (uint16_t)(EndPoint[dfifo].rxCounter + len);
	
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Прочитать установочный пакет EP0*/

void Read_Setup_Fifo()
{
	/*8 байт!!!*/
	setup_pkt_data.raw_data[0] = USB_OTG_DFIFO(0);
	setup_pkt_data.raw_data[1] = USB_OTG_DFIFO(0);
}

/*-----------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------*/
/*Обработчик сброса USB*/

void Enumerate_Reset()
{
	device_state = DEVICE_STATE_RESET;  
	USB_OTG_FS->GINTSTS &= ~0xFFFFFFFF;
	
	USB_OTG_DEVICE->DAINTMSK = 0x30003;
	 /*Демаскировать прерывания IEPM, OEPM для EP0, EP1, IEPM для EP2*/
	USB_OTG_DEVICE->DOEPMSK  = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM; /*Снятие маски, фаза НАСТРОЙКИ завершена, маска, передача завершена, прерывание для ВЫХОДА*/
	USB_OTG_DEVICE->DIEPMSK  = USB_OTG_DIEPMSK_XFRCM; /*Прерывание TransferR Completed для IN*/

	USB_OTG_DEVICE->DCFG  &= ~USB_OTG_DCFG_DAD_Msk;  /*перед перечислением задать адрес 0*/

	/*EP1*/
	USB_EP_IN(1)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |	
				USB_OTG_DIEPCTL_TXFNUM_0 |  /* TX Number 1 */
				USB_OTG_DIEPCTL_EPTYP_1 |  /* Eptype 10 means Bulk */
				USB_OTG_DIEPCTL_USBAEP |  /* Set Endpoint active */
				USB_CDC_MAX_PACKET_SIZE;  /* Max Packet size (bytes) */ 
	
	USB_EP_OUT(1)->DOEPCTL = USB_OTG_DOEPCTL_EPENA | 	/* Enable Endpoint */
				USB_OTG_DOEPCTL_CNAK |  /* Clear NAK */
				USB_OTG_DOEPCTL_EPTYP_1 |  /* Eptype 10 means Bulk */
				USB_OTG_DOEPCTL_USBAEP | /* Set Endpoint active */
				USB_CDC_MAX_PACKET_SIZE; /* CHK MPSIZ The application must program this field with the maximum packet size for the current logical endpoint. This value is in bytes */

	USB_EP_OUT(1)->DOEPTSIZ = 0;			
	USB_EP_OUT(1)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (DOEPT_TRANSFER_PCT << USB_OTG_DOEPTSIZ_PKTCNT_Pos)); /* RM quote: Indicates the total number of USB packets that constitute the Transfer Size amount of data for this endpoint. This field is decremented every time a packet (maximum size or short packet) is written to the RxFIFO */

	USB_EP_OUT(1)->DOEPTSIZ |= DOEPT_TRANSFER_SIZE; /* Transfer size. If you set transfer size = max. packet, the core will interrupt the application at the end of each packet */
	
	/*EP2*/
	USB_EP_IN(2)->DIEPCTL = USB_OTG_DIEPCTL_SNAK |
				USB_OTG_DIEPCTL_TXFNUM_1 | 
				USB_OTG_DIEPCTL_EPTYP |  // Eptype 11 means Interrupt EP 
				USB_OTG_DIEPCTL_USBAEP | 
				0x08; 
																									
}

/*-----------------------------------------------------------------------------------------------*/


