import numpy as np
import os

LOCATIONS_FILE = 'locations_test_09_20.csv'
OUT_FILE = 'submit_0920.csv'
# specify data locations
datapath = '../data'  # relative to python script
max_hstd = 0.5

lowQualityRides = [
    '2022-06-28-20-56-us-ca-sjc-r/samsunga32',
    '2022-10-06-20-46-us-ca-sjc-r/sm-a205u']

datapath = os.path.abspath(datapath)
os.chdir(datapath)

# load baseline data 
baseline_file = 'best_submission.csv'
base_txt = np.genfromtxt(baseline_file, delimiter=',',invalid_raise=False, 
                         skip_header=1, dtype=str)
msecs_base = base_txt[:,1].astype(np.int64)
phones_base = base_txt[:,0]
pos_base = base_txt[:,2:4].astype(float)

# load test data
d = np.genfromtxt(LOCATIONS_FILE, delimiter=',',invalid_raise=False, skip_header=1, dtype=str)
stds = d[:,7:10].astype(float)
hstds = np.sqrt(stds[:,0]**2 + stds[:,1]**2)

        
# merge low quality rides with Google baseline
for trip_phone in np.unique(d[:,0]):
    if trip_phone in lowQualityRides:
        ixt = np.where(d[:,0] == trip_phone)[0]
        #ix = ixt[np.where(d[ixt,5] != '2')[0]]
        ix = ixt[np.where(hstds[ixt] >= max_hstd)[0]]
        d[ix,2:4] = pos_base[ix,0:2]

# save results to file
fout =open( OUT_FILE,'w')
fout.write('tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n')
for i in range(len(d)):
    # write results to combined file
    fout.write('%s, %s, %3.12f, %3.12f\n' % (d[i,0], d[i,1], float(d[i,2]), float(d[i,3])))
fout.close()