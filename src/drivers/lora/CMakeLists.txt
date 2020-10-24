px4_add_module(
	MODULE drivers__lora
	MAIN lora
	COMPILE_FLAGS
		-Wno-shadow -Wno-unused-variable
		-DREGION_EU868 -DACTIVE_REGION=LORAMAC_REGION_EU868
	INCLUDES
		vendor
		vendor/mac
	SRCS
		lora.cpp
		support.cpp
		vendor/sx126x/radio.c
		vendor/sx126x/sx126x.c
		vendor/timer.c
		vendor/rtc-board.cpp
		vendor/mac/LoRaMac.c
		vendor/mac/LoRaMacAdr.c
		vendor/mac/LoRaMacClassB.c
		vendor/mac/LoRaMacCommands.c
		vendor/mac/LoRaMacConfirmQueue.c
		vendor/mac/LoRaMacCrypto.c
		vendor/mac/LoRaMacParser.c
		vendor/mac/LoRaMacSerializer.c
		vendor/mac/region/Region.c
		vendor/mac/region/RegionAS923.c
		vendor/mac/region/RegionAU915.c
		vendor/mac/region/RegionCN470.c
		vendor/mac/region/RegionCN779.c
		vendor/mac/region/RegionCommon.c
		vendor/mac/region/RegionEU433.c
		vendor/mac/region/RegionEU868.c
		vendor/mac/region/RegionIN865.c
		vendor/mac/region/RegionKR920.c
		vendor/mac/region/RegionRU864.c
		vendor/mac/region/RegionUS915.c
		vendor/soft-se/aes.c
		vendor/soft-se/cmac.c
		vendor/soft-se/soft-se.c
	)
