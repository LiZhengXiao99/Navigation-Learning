""" 
get_base_data.py - retrieve base observation and navigation data for the
    2023 GSDC competition 
"""

import os
from os.path import join
from datetime import datetime
import numpy as np
import requests
import gzip
from glob import glob
import subprocess

# Input parameters
datadir = '../data/test'  # relative to python script
# List of CORS stations to use
stas = ['slac', 'vdcy', 'p222']  # Bay Area, LA, backup for Bay Area

# site to retrieve base observation data
obs_url_base = 'https://geodesy.noaa.gov/corsdata/rinex'  

# site to retrieve satellite navigation data
nav_url_base = 'https://cddis.nasa.gov/archive/gnss/data/daily' #/2021/342/21p/ 
nav_file_base = 'BRDM00DLR_S_' # 20213420000_01D_MN.rnx.gz
# Access to CDDIS navigation data requires registering for a free account and 
# setup of a .netrc file as described at 
# https://cddis.nasa.gov/Data_and_Derived_Products/CreateNetrcFile.html.  
# Make sure this file  is in the users home directory 

# Make sure you have downloaded this executable before running this code
crx2rnx_bin = '../../rtklib/CRX2RNX' # relative to data directory

# Loop through data sets in the data directory
os.chdir(datadir)
for dataset in np.sort(os.listdir()):
    if not os.path.isdir(join(dataset)):
        continue
    print(dataset)
    ymd = dataset.split('-')
    doy = datetime(int(ymd[0]), int(ymd[1]), int(ymd[2])).timetuple().tm_yday # get day of year
    doy = str(doy).zfill(3)
    
    if len(glob(join(dataset,'*.*o'))) == 0:
        # get obs data
        i = 1 if '-lax-' in dataset else 0  # use different base for LA
        fname = stas[i] + doy + '0.' + ymd[0][2:4] + 'd.gz'
        url = '/'.join([obs_url_base, ymd[0], doy, stas[i], fname])
        try:
            obs = gzip.decompress(requests.get(url).content) # get obs and decompress
            # write obs data
            open(join(dataset, fname[:-3]), "wb").write(obs)
        except:
            # try backup CORS station
            print('Try backup CORS:', dataset)
            i += 2
            fname = stas[i] + doy + '0.' + ymd[0][2:4] + 'd.gz'
            url = '/'.join([obs_url_base, ymd[0], doy, stas[i], fname])
            try:
                obs = gzip.decompress(requests.get(url).content) # get obs and decompress
                # write obs data
                open(join(dataset, fname[:-3]), "wb").write(obs)
            except:
                print('Fail obs: %s' % dataset)
            
        # convert compact rinex to rinex
        crx_files = glob(join(dataset,'*.*d'))
        if len(crx_files) > 0:
            subprocess.call([crx2rnx_bin, '-f', crx_files[0]])
    
    # get nav data
    if len(glob(join(dataset,'*.rnx'))) > 0:
           continue  # file already exists
    fname = nav_file_base + ymd[0] + doy + '0000_01D_MN' + '.rnx.gz'
    url = '/'.join([nav_url_base, ymd[0], doy, ymd[0][2:4]+'p', fname])
    try:
        obs = gzip.decompress(requests.get(url).content) # get obs and decompress    
        # write nav data
        open(join(dataset, fname[:-3]), "wb").write(obs)
    except:
        print('Fail nav: %s' % dataset)