"""
create_groundtruth_csv.py - create csv file from all ground truth files
"""

import os
from os.path import join, isfile


datapath = '../data/train' # relative to python script
# Select phones to process
# all phones
# PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel5a', 'pixel6pro', 'pixel7pro',
#           'mi8', 'xiaomimi8',
#           'sm-g988b', 'sm-g955f', 'sm-s908b', 'sm-a226b', 'sm-a600t',
#           'sm-a505g', 'sm-a325f', 'sm-a217m', 'sm-a205u', 'sm-a505u', 
#           'samsungs22ultra', 'samsunga325g', 'samsunga32', 'samsung21ultra']

# just phones in test set
PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel6pro', 'pixel7pro',
          'mi8', 'xiaomimi8',
          'sm-g988b', 'sm-s908b', 'sm-a325f', 'sm-a505u', 
          'samsunga325g', 'samsunga32']
GPS_TO_UTC = 315964782  # second

# open output file
datapath = os.path.abspath(datapath)
os.chdir(datapath)
fout =open('../ground_truths_train.csv','w')
fout.write('tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees, Height, Heading\n')

# get list of data sets in data path
datasets = sorted(os.listdir(datapath))

# loop through data set folders
for dataset in datasets:
    if isfile(dataset):
        continue
    try:
        phones = sorted(PHONES)
    except:
        phones = os.listdir(join(datapath,dataset))
    for phone in phones:
        folder = join(datapath, dataset, phone)
        if isfile(folder):
            continue
        
        csv_file = join(folder, 'ground_truth.csv')
        if not isfile(csv_file):
            continue

        # parse ground truth file
        with open(csv_file) as f:
            lines = f.readlines()[1:]
        flag = 0
        for line in lines:
            if len(line) <= 1:
                continue
            d = line.split(',')
            t = float(d[8]) # get time stamp
            if flag == 0:            
                print('%20s,%16s' % (dataset, phone))
                flag = 1
            # write results to combined file
            fout.write('%s/%s,%.0f,%s,%s,%s, %s\n' % ((dataset, phone, t, d[2],
                                                      d[3], d[4][:7], d[7][:5])))
        
fout.close()