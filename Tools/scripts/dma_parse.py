#!/usr/bin/env python
'''
extra DMA mapping tables from a stm32 datasheet

This assumes a csv file extracted from the datasheet using tablula:
 https://github.com/tabulapdf/tabula

'''

import sys, csv, os

def parse_dma_table(fname, dma_num, table):
    csvt = csv.reader(open(fname,'rb'))
    i = 0
    for row in csvt:
        if not row[0].startswith('Channel '):
            continue
        channel = int(row[0].split(' ')[1])
        for stream in range(8):
            s = row[stream+1]
            s = s.replace('_\r', '_')
            s = s.replace('\r_', '_')
            if s == '-':
                continue
            keys = s.split()
            for k in keys:
                brace = k.find('(')
                if brace != -1:
                    k = k[:brace]
                if k not in table:
                    table[k] = []
                table[k] += [(dma_num, stream)]

table = {}

if len(sys.argv) != 3:
    print("Error: expected 2 CSV files (one for each DMA)")
    sys.exit(1)

parse_dma_table(sys.argv[1], 1, table)
parse_dma_table(sys.argv[2], 2, table)

sys.stdout.write("DMA_map = {\n");
sys.stdout.write('\t# format is [DMA_TABLE, StreamNum]\n')
sys.stdout.write('\t# extracted from %s and %s\n' % (
    os.path.basename(sys.argv[1]), os.path.basename(sys.argv[2])))
for k in sorted(table.iterkeys()):
    sys.stdout.write('\t"%s"\t:\t[' % k)
    for i in range(len(table[k])):
        sys.stdout.write("[%u,%u]" % (table[k][i][0], table[k][i][1]))
        if i < len(table[k])-1:
            sys.stdout.write(",")
    sys.stdout.write("],\n")
sys.stdout.write("}\n");
