
STM32F427_DMA_Map = {
	"SPI1_RX"	:	[[2,0],[2,2]],
	"SPI1_TX"	:	[[2,3],[2,5]],
	"SPI2_RX"	:	[[1,3]],
	"SPI2_TX"	:	[[1,4]],
	"SPI3_RX"	:	[[1,0],[1,2]],
	"SPI3_TX"	:	[[1,5],[1,7]],
	"SPI4_RX"	:	[[2,0],[2,3]],
	"SPI4_TX"	:	[[2,1],[2,4]],
	"USART1_RX"	:	[[2,2],[2,5]],
	"USART1_TX"	:	[[2,7]],
	"USART2_RX"	:	[[1,5]],
	"USART2_TX"	:	[[1,6]],
	"USART3_RX"	:	[[1,1]],
	"USART3_TX"	:	[[1,3],[1,4]],
	"UART4_RX"	:	[[1,2]],
	"UART4_TX"	:	[[1,4]],
	"UART5_RX"	:	[[1,0]],
	"UART5_TX"	:	[[1,7]],
	"USART6_RX" :	[[2,1],[2,2]],
	"USART6_TX" :	[[2,6],[2,7]],
	"UART7_RX"	:	[[1,3]],
	"UART7_TX"	:	[[1,1]],
	"UART8_RX"	:	[[1,6]],
	"UART8_TX"	:	[[1,0]],
	"ADC1"		:	[[1,0],[1,4]],
	"ADC2"		:	[[1,2],[1,3]],
	"ADC3"		:	[[1,0],[1,1]],
	"I2C1_RX"	:	[[1,0],[1,5]],
	"I2C1_TX"	:	[[1,6],[1,7]],
	"I2C2_RX"	:	[[1,2],[1,3]],
	"I2C2_TX"	:	[[1,7]],
	"I2C3_RX"	:	[[1,2]],
	"I2C3_TX"	:	[[1,4]]
}

STM32F412_DMA_Map = {
	"SPI1_RX"	:	[[2,0],[2,2]],
	"SPI1_TX"	:	[[2,2],[2,3],[2,5]],
	"SPI2_RX"	:	[[1,3]],
	"SPI2_TX"	:	[[1,4]],
	"SPI3_RX"	:	[[1,0],[1,2]],
	"SPI3_TX"	:	[[1,5],[1,7]],
	"SPI4_RX"	:	[[2,0],[2,4],[2,3]],
	"SPI4_TX"	:	[[2,1],[2,4]],
	"SPI5_RX"	:	[[2,3],[2,5]],
	"SPI5_TX"	:	[[2,4],[2,5],[2,6]],
	"USART1_RX"	:	[[2,2],[2,5]],
	"USART1_TX"	:	[[2,7]],
	"USART2_RX"	:	[[1,5],[1,7]],
	"USART2_TX"	:	[[1,6]],
	"USART3_RX"	:	[[1,1]],
	"USART3_TX"	:	[[1,3],[1,4]],
	"USART6_RX" :	[[2,1],[2,2]],
	"USART6_TX" :	[[2,6],[2,7]],
	"ADC1"		:	[[2,0],[2,4]],
	"I2C1_RX"	:	[[1,0],[1,5]],
	"I2C1_TX"	:	[[1,1],[1,6],[1,7]],
	"I2C2_RX"	:	[[1,2],[1,3]],
	"I2C2_TX"	:	[[1,7]],
	"I2C3_RX"	:	[[1,1],[1,2]],
	"I2C3_TX"	:	[[1,4],[1,5]],
}

PERIPHONDMA_LIST = ["ADC1","SPI1_RX","SPI1_TX","SPI2_RX","SPI2_TX","SPI5_RX","SPI5_TX",\
					"I2C1_RX","I2C1_TX","USART3_TX","USART3_RX","USART6_TX","USART6_RX"]

def check_possibility(periph, dma_stream, curr_dict, dma_map, check_list):
	for other_periph in curr_dict:
		if other_periph != periph:
			if curr_dict[other_periph] == dma_stream:
				ignore_list.append(periph)
				check_str = "%s(%d,%d) %s(%d,%d)" % (
										other_periph,
										curr_dict[other_periph][0],
										curr_dict[other_periph][1],
										periph,
										dma_stream[0],
										dma_stream[1])
				#check if we did this before
				if check_str in check_list:
					return False
				check_list.append(check_str)
				print "Trying to Resolve Conflict: ", check_str
				#check if we can resolve by swapping with other periphs
				for stream in dma_map[other_periph]:
					if stream != curr_dict[other_periph] and \
					   check_possibility(other_periph, stream, curr_dict, dma_map, check_list):
						curr_dict[other_periph] = stream
						return True
				return False
	return True
unassigned = []
curr_dict = {}
dma_map = STM32F412_DMA_Map
for periph in PERIPHONDMA_LIST:
	assigned = False
	ignore_list = []
	check_list = []
	for stream in dma_map[periph]:
		if check_possibility(periph, stream, curr_dict, dma_map, check_list):
			curr_dict[periph] = stream
			assigned = True
			break
	if assigned == False:
		unassigned.append(periph)

print "\n\nMOST VIABLE DMA CONFIG:\n"
print "{:<15} {:<15}".format('Peripherals','Stream')
for key in sorted(curr_dict.iterkeys()):
	print "{:<15} {:<15}".format(key,curr_dict[key])

print "\n\nFollowing Peripherals can't be resolved:", unassigned