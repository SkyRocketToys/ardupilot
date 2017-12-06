#!/usr/bin/env python

import optparse, sys, fnmatch

boards = ['F412', 'FMUv3']

parser = optparse.OptionParser("dma_resolver.py")
parser.add_option("-B", "--board", type='choice', default=None, choices=boards, help='board type')

opts, args = parser.parse_args()

if opts.board is None:
        print("Please choose a board from: %s" % boards)
        sys.exit(1)

STM32F427_DMA_Map = {
	# format is [DMA_TABLE, StreamNum]
	# extracted from tabula-STM324x7-306.csv and tabula-STM324x7-307.csv
	"ADC1"		:	[(2,0),(2,4)],
	"ADC2"		:	[(2,2),(2,3)],
	"ADC3"		:	[(2,0),(2,1)],
	"CRYP_IN"	:	[(2,6)],
	"CRYP_OUT"	:	[(2,5)],
	"DAC1"		:	[(1,5)],
	"DAC2"		:	[(1,6)],
	"DCMI"		:	[(2,1),(2,7)],
	"HASH_IN"	:	[(2,7)],
	"I2C1_RX"	:	[(1,0),(1,5)],
	"I2C1_TX"	:	[(1,6),(1,7)],
	"I2C2_RX"	:	[(1,2),(1,3)],
	"I2C2_TX"	:	[(1,7)],
	"I2C3_RX"	:	[(1,2)],
	"I2C3_TX"	:	[(1,4)],
	"I2S2_EXT_RX"	:	[(1,3)],
	"I2S2_EXT_TX"	:	[(1,4)],
	"I2S3_EXT_RX"	:	[(1,2),(1,0)],
	"I2S3_EXT_TX"	:	[(1,5)],
	"SAI1_A"	:	[(2,1),(2,3)],
	"SAI1_B"	:	[(2,5),(2,4)],
	"SDIO"		:	[(2,3),(2,6)],
	"SPI1_RX"	:	[(2,0),(2,2)],
	"SPI1_TX"	:	[(2,3),(2,5)],
	"SPI2_RX"	:	[(1,3)],
	"SPI2_TX"	:	[(1,4)],
	"SPI3_RX"	:	[(1,0),(1,2)],
	"SPI3_TX"	:	[(1,5),(1,7)],
	"SPI4_RX"	:	[(2,0),(2,3)],
	"SPI4_TX"	:	[(2,1),(2,4)],
	"SPI5_RX"	:	[(2,3),(2,5)],
	"SPI5_TX"	:	[(2,4),(2,6)],
	"SPI6_RX"	:	[(2,6)],
	"SPI6_TX"	:	[(2,5)],
	"TIM1_CH1"	:	[(2,6),(2,1),(2,3)],
	"TIM1_CH2"	:	[(2,6),(2,2)],
	"TIM1_CH3"	:	[(2,6),(2,6)],
	"TIM1_CH4"	:	[(2,4)],
	"TIM1_COM"	:	[(2,4)],
	"TIM1_TRIG"	:	[(2,0),(2,4)],
	"TIM1_UP"	:	[(2,5)],
	"TIM2_CH1"	:	[(1,5)],
	"TIM2_CH2"	:	[(1,6)],
	"TIM2_CH3"	:	[(1,1)],
	"TIM2_CH4"	:	[(1,6),(1,7)],
	"TIM2_UP"	:	[(1,1),(1,7)],
	"TIM3_CH1"	:	[(1,4)],
	"TIM3_CH2"	:	[(1,5)],
	"TIM3_CH3"	:	[(1,7)],
	"TIM3_CH4"	:	[(1,2)],
	"TIM3_TRIG"	:	[(1,4)],
	"TIM3_UP"	:	[(1,2)],
	"TIM4_CH1"	:	[(1,0)],
	"TIM4_CH2"	:	[(1,3)],
	"TIM4_CH3"	:	[(1,7)],
	"TIM4_UP"	:	[(1,6)],
	"TIM5_CH1"	:	[(1,2)],
	"TIM5_CH2"	:	[(1,4)],
	"TIM5_CH3"	:	[(1,0)],
	"TIM5_CH4"	:	[(1,1),(1,3)],
	"TIM5_TRIG"	:	[(1,1),(1,3)],
	"TIM5_UP"	:	[(1,0),(1,6)],
	"TIM6_UP"	:	[(1,1)],
	"TIM7_UP"	:	[(1,2),(1,4)],
	"TIM8_CH1"	:	[(2,2),(2,2)],
	"TIM8_CH2"	:	[(2,2),(2,3)],
	"TIM8_CH3"	:	[(2,2),(2,4)],
	"TIM8_CH4"	:	[(2,7)],
	"TIM8_COM"	:	[(2,7)],
	"TIM8_TRIG"	:	[(2,7)],
	"TIM8_UP"	:	[(2,1)],
	"UART4_RX"	:	[(1,2)],
	"UART4_TX"	:	[(1,4)],
	"UART5_RX"	:	[(1,0)],
	"UART5_TX"	:	[(1,7)],
	"UART7_RX"	:	[(1,3)],
	"UART7_TX"	:	[(1,1)],
	"UART8_RX"	:	[(1,6)],
	"UART8_TX"	:	[(1,0)],
	"USART1_RX"	:	[(2,2),(2,5)],
	"USART1_TX"	:	[(2,7)],
	"USART2_RX"	:	[(1,5)],
	"USART2_TX"	:	[(1,6)],
	"USART3_RX"	:	[(1,1)],
	"USART3_TX"	:	[(1,3),(1,4)],
	"USART6_RX"	:	[(2,1),(2,2)],
	"USART6_TX"	:	[(2,6),(2,7)],
}

STM32F412_DMA_Map = {
	# format is [DMA_TABLE, StreamNum]
	# extracted from tabula-stm32f412-ref-manual-196.csv and tabula-stm32f412-ref-manual-196(1).csv
	"ADC1"		:	[(2,0),(2,4)],
	"DFSDM1_FLT0"	:	[(2,6),(2,0)],
	"DFSDM1_FLT1"	:	[(2,1),(2,4)],
	"I2C1_RX"	:	[(1,0),(1,5)],
	"I2C1_TX"	:	[(1,1),(1,6),(1,7)],
	"I2C2_RX"	:	[(1,2),(1,3)],
	"I2C2_TX"	:	[(1,7)],
	"I2C3_RX"	:	[(1,1),(1,2)],
	"I2C3_TX"	:	[(1,4),(1,5)],
	"I2CFMP1_RX"	:	[(1,3),(1,0)],
	"I2CFMP1_TX"	:	[(1,1),(1,7)],
	"I2S2EXT_RX"	:	[(1,3)],
	"I2S2_EXT_TX"	:	[(1,4)],
	"I2S3_EXT_RX"	:	[(1,2),(1,0)],
	"I2S3_EXT_TX"	:	[(1,5)],
	"QUADSPI"	:	[(2,7)],
	"SDIO"		:	[(2,3),(2,6)],
	"SPI1_RX"	:	[(2,0),(2,2)],
	"SPI1_TX"	:	[(2,2),(2,3),(2,5)],
	"SPI2_RX"	:	[(1,3)],
	"SPI2_TX"	:	[(1,4)],
	"SPI3_RX"	:	[(1,0),(1,2)],
	"SPI3_TX"	:	[(1,5),(1,7)],
	"SPI4_RX"	:	[(2,0),(2,4),(2,3)],
	"SPI4_TX"	:	[(2,1),(2,4)],
	"SPI5_RX"	:	[(2,3),(2,5)],
	"SPI5_TX"	:	[(2,4),(2,5),(2,6)],
	"TIM1_CH1"	:	[(2,6),(2,1),(2,3)],
	"TIM1_CH2"	:	[(2,6),(2,2)],
	"TIM1_CH3"	:	[(2,6),(2,6)],
	"TIM1_CH4"	:	[(2,4)],
	"TIM1_COM"	:	[(2,4)],
	"TIM1_TRIG"	:	[(2,0),(2,4)],
	"TIM1_UP"	:	[(2,5)],
	"TIM2_CH1"	:	[(1,5)],
	"TIM2_CH2"	:	[(1,6)],
	"TIM2_CH3"	:	[(1,1)],
	"TIM2_CH4"	:	[(1,6),(1,7)],
	"TIM2_UP"	:	[(1,1),(1,7)],
	"TIM3_CH1"	:	[(1,4)],
	"TIM3_CH2"	:	[(1,5)],
	"TIM3_CH3"	:	[(1,7)],
	"TIM3_CH4"	:	[(1,2)],
	"TIM3_TRIG"	:	[(1,4)],
	"TIM3_UP"	:	[(1,2)],
	"TIM4_CH1"	:	[(1,0)],
	"TIM4_CH2"	:	[(1,3)],
	"TIM4_CH3"	:	[(1,7)],
	"TIM4_UP"	:	[(1,6)],
	"TIM5_CH1"	:	[(1,2)],
	"TIM5_CH2"	:	[(1,4)],
	"TIM5_CH3"	:	[(1,0)],
	"TIM5_CH4"	:	[(1,1),(1,3)],
	"TIM5_TRIG"	:	[(1,1),(1,3)],
	"TIM5_UP"	:	[(1,0),(1,6)],
	"TIM6_UP"	:	[(1,1)],
	"TIM7_UP"	:	[(1,2),(1,4)],
	"TIM8_CH1"	:	[(2,2),(2,2)],
	"TIM8_CH2"	:	[(2,2),(2,3)],
	"TIM8_CH3"	:	[(2,2),(2,4)],
	"TIM8_CH4"	:	[(2,7)],
	"TIM8_COM"	:	[(2,7)],
	"TIM8_TRIG"	:	[(2,7)],
	"TIM8_UP"	:	[(2,1)],
	"USART1_RX"	:	[(2,2),(2,5)],
	"USART1_TX"	:	[(2,7)],
	"USART2_RX"	:	[(1,5),(1,7)],
	"USART2_TX"	:	[(1,6)],
	"USART3_RX"	:	[(1,1)],
	"USART3_TX"	:	[(1,3),(1,4)],
	"USART6_RX"	:	[(2,1),(2,2)],
	"USART6_TX"	:	[(2,6),(2,7)],
}

# peripheral types that can be shared, wildcard patterns
SHARED_MAP = [ "I2C*", "USART*_TX", "UART*_TX", "SPI*" ]

if opts.board == 'FMUv3':
        dma_map = STM32F427_DMA_Map
        PERIPHONDMA_LIST = ["SDIO"]
        PERIPHONDMA_LIST += ["ADC1"]
        PERIPHONDMA_LIST += ["SPI1_RX","SPI1_TX","SPI2_RX","SPI2_TX","SPI4_RX","SPI4_TX"]
        PERIPHONDMA_LIST += ["I2C1_RX","I2C1_TX","I2C2_RX","I2C2_TX"]
        PERIPHONDMA_LIST += ["USART1_TX","USART1_RX"]
        PERIPHONDMA_LIST += ["USART2_TX","USART2_RX"]
        PERIPHONDMA_LIST += ["USART3_TX","USART3_RX"]
        PERIPHONDMA_LIST += ["UART4_TX","UART4_RX"]
        PERIPHONDMA_LIST += ["USART6_TX","USART6_RX"]
        PERIPHONDMA_LIST += ["UART7_TX","UART7_RX"]
        PERIPHONDMA_LIST += ["UART8_TX","UART8_RX"]
elif opts.board == 'F412':
        dma_map = STM32F412_DMA_Map
        PERIPHONDMA_LIST = ["ADC1"]
        PERIPHONDMA_LIST += ["SPI1_RX","SPI1_TX","SPI2_RX","SPI2_TX","SPI5_RX","SPI5_TX"]
        PERIPHONDMA_LIST += ["I2C1_RX","I2C1_TX","I2C2_RX", "I2C2_TX"]
        PERIPHONDMA_LIST += ["USART3_TX","USART3_RX"]
        PERIPHONDMA_LIST += ["USART6_TX","USART6_RX"]
        PERIPHONDMA_LIST += ["USART2_TX","USART2_RX"]


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
				print("Trying to Resolve Conflict: ", check_str)
				#check if we can resolve by swapping with other periphs
				for stream in dma_map[other_periph]:
					if stream != curr_dict[other_periph] and \
					   check_possibility(other_periph, stream, curr_dict, dma_map, check_list):
						curr_dict[other_periph] = stream
						return True
				return False
	return True

def can_share(periph):
        '''check if a peripheral is in the SHARED_MAP list'''
        for f in SHARED_MAP:
                if fnmatch.fnmatch(periph, f):
                        return True
        print("%s can't share" % periph)
        return False
        

unassigned = []
curr_dict = {}

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

# now look for shared DMA possibilities
stream_assign = {}
for k in curr_dict.iterkeys():
        stream_assign[curr_dict[k]] = [k]

print("Unassigned before sharing: %s" % unassigned)
unassigned_new = unassigned[:]
for periph in unassigned:
        print("Checking sharing for %s" % periph)
	for stream in dma_map[periph]:
                share_ok = True
                for periph2 in stream_assign[stream]:
                        if not can_share(periph) or not can_share(periph2):
                                share_ok = False
                if share_ok:
                        print("Sharing %s on %s with %s" % (periph, stream, stream_assign[stream]))
                        curr_dict[periph] = stream
                        stream_assign[stream].append(periph)
                        unassigned_new.remove(periph)
                        break
unassigned = unassigned_new


def chibios_dma_define_name(key):
        '''return define name needed for board.h for ChibiOS'''
        if key.startswith('ADC'):
                return 'STM32_ADC_%s_DMA_STREAM' % key
        elif key.startswith('SPI'):
                return 'STM32_SPI_%s_DMA_STREAM' % key
        elif key.startswith('I2C'):
                return 'STM32_I2C_%s_DMA_STREAM' % key
        elif key.startswith('USART'):
                return 'STM32_UART_%s_DMA_STREAM' % key
        elif key.startswith('UART'):
                return 'STM32_UART_%s_DMA_STREAM' % key
        elif key.startswith('SDIO'):
                return 'STM32_SDC_%s_DMA_STREAM' % key
        else:
                print("Error: Unknown key type %s" % key)
                sys.exit(1)
                

print("\n\nMOST VIABLE DMA CONFIG:\n")
print("// auto-generated DMA mapping from dma_resolver.py")
for key in sorted(curr_dict.iterkeys()):
        stream = curr_dict[key]
        shared = ''
        if len(stream_assign[stream]) > 1:
                shared = ' // shared %s' % ','.join(stream_assign[stream])
        print("#define %-30s STM32_DMA_STREAM_ID(%u, %u)%s" % (chibios_dma_define_name(key),
                                                               curr_dict[key][0],
                                                               curr_dict[key][1],
                                                               shared))

if unassigned:
        print("\n// The following Peripherals can't be resolved: %s" % unassigned)

# now generate UARTDriver.cpp config lines
print("\n\n// generated UART configuration lines")
for u in range(1,9):
        key = None
        if 'USART%u_TX' % u in PERIPHONDMA_LIST:
                key = 'USART%u' % u
        if 'UART%u_TX' % u in PERIPHONDMA_LIST:
                key = 'UART%u' % u
        if 'USART%u_RX' % u in PERIPHONDMA_LIST:
                key = 'USART%u' % u
        if 'UART%u_RX' % u in PERIPHONDMA_LIST:
                key = 'UART%u' % u
        if key is None:
                continue
        sys.stdout.write("#define %s_CONFIG { (BaseSequentialStream*) &SD%u, false, " % (key, u))
        if key + "_RX" in curr_dict:
                sys.stdout.write("true, STM32_UART_%s_RX_DMA_STREAM, STM32_%s_RX_DMA_CHN, " % (key, key))
        else:
                sys.stdout.write("false, 0, 0, ")
        if key + "_TX" in curr_dict:
                sys.stdout.write("true, STM32_UART_%s_TX_DMA_STREAM, STM32_%s_TX_DMA_CHN}\n" % (key, key))
        else:
                sys.stdout.write("false, 0, 0}\n")
