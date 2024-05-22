""" create_baseline_csv_from_pos.py -  Create csv file PPK solution files using timestamps in reference file
"""

import os
from os.path import join, isfile
import numpy as np
from datetime import date

########### Input parameters ###############################

DATA_SET = 'test'
SOL_TAG = '_rtklib'
datapath = '../data' # relative to python script
rovfile = 'gnss_log'
hdrlen = 25    # 25 for RTKLIB, 1 for RTKLIB-py

outThresh = 100   # max horizontal accuracy estimate
# phones in test set
PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel6pro', 'pixel7pro',
          'mi8', 'xiaomimi8',
          'sm-g988b', 'sm-s908b', 'sm-a325f', 'sm-a505u', 'sm-a205u',
          'samsunga325g', 'samsunga32']
# PHONES = []  # use all phones

# Also make sure the appropriate reference file is in the datapath
#  test: best_submission.csv - best available sample submission
# train: ground_truths_train.csv - created with create_ground_truths.py

############################################################

GPS_TO_UTC = 315964782  # second

def create_csv(datapath, DATA_SET, SOL_TAG):
    # get timestamps from existing baseline file
    datapath = os.path.abspath(datapath)
    os.chdir(datapath)
    if DATA_SET[:5] == 'train':
        baseline_file = 'ground_truths_' + DATA_SET + '.csv'
    else: # 'test'
        baseline_file = 'best_submission.csv'
    # read data from baseline file
    base_txt = np.genfromtxt(baseline_file, delimiter=',',invalid_raise=False, 
                             skip_header=1, dtype=str)
    msecs_base = base_txt[:,1].astype(np.int64)
    phones_base = base_txt[:,0]
    pos_base = base_txt[:,2:4].astype(float) # baseline positions
    
    # open output file
    fout =open('locations_' + DATA_SET + '_' + date.today().strftime("%m_%d") + '.csv','w')
    fout.write('tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n')
    
    # get list of data sets in data path
    os.chdir(join(datapath, DATA_SET))
    trips = np.sort(os.listdir())
    
    # loop through data set folders
    ix_b, npts = [], 0
    for trip in trips:
        if isfile(trip):
            continue
        phones = os.listdir(trip)
        # loop through phone folders
        for phone in phones:
            if isinstance(phone, bytearray):
                phone = phone.decode('utf-8')
            # check for valid folder and file
            folder = join(trip, phone)
            if isfile(folder):
                continue
            if PHONES != [] and phone not in PHONES:
                continue
            trip_phone = trip + '/' + phone
            #print(trip_phone)
    
            ix_b = np.where(phones_base == trip_phone)[0]
            sol_path = join(folder, 'supplemental', rovfile + SOL_TAG + '.pos')
            fields = []
            if isfile(sol_path):
                # parse solution file
                fields = np.genfromtxt(sol_path, invalid_raise=False, skip_header=hdrlen)
            if len(fields) > 1:
                if int(fields[0,1]) > int(fields[-1,1]): # invert if backwards solution
                    fields = fields[::-1]
                pos = fields[:,2:5]
                qs = fields[:,5].astype(int)
                nss = fields[:,6].astype(int)
                acc = fields[:,7:10]
                msecs = (1000 * (fields[:,0] * 7 * 24 * 3600 + fields[:,1])).astype(np.int64)
                msecs += GPS_TO_UTC * 1000
            # if no data, use baseline data
            if not isfile(sol_path) or len(fields) == 0:
                print('Warning: data substitution: ', sol_path)
                msecs = msecs_base[ix_b].copy()
                pos = acc = np.zeros((len(msecs), 3))
                pos[:,:2] = pos_base[ix_b].copy()
                qs = nss = np.zeros(len(msecs))
           
            # interpolate to baseline timestamps to fill in missing samples
            llhs = []; stds = []
            for j in range(6):
                if j < 3:
                    llhs.append(np.interp(msecs_base[ix_b], msecs, pos[:,j]))
                    stds.append(np.interp(msecs_base[ix_b], msecs, acc[:,j]))
            qsi = np.interp(msecs_base[ix_b], msecs, qs)
            nssi = np.interp(msecs_base[ix_b], msecs, nss)
    
            # write results to combined file
            for i in range(len(ix_b)):
                fout.write('%s,%d,%.12f,%.12f,%.2f,%.0f,%.0f,%.3f,%.3f,%.3f\n' % 
                        (trip_phone, msecs_base[ix_b[i]], llhs[0][i], llhs[1][i],
                         llhs[2][i], qsi[i], nssi[i], stds[0][i], stds[1][i], 
                         stds[2][i]))
                try:
                    npts += len(fields)
                except:
                    pass
    
    fout.close()
    return npts

if __name__ == '__main__':
    create_csv(datapath, DATA_SET, SOL_TAG)