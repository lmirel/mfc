#!/usr/bin/python

#
# sudo python miniapp.py
#
import os
import threading
import time
import pexpect
from flask import Flask, render_template, flash, request, send_file
from wtforms import Form, TextField, TextAreaField, validators, StringField, SubmitField

#flag variables
_mfcsvr     = False #platform mover server
_mfccli     = False #wheel client extractor
_mfcnpcars  = False #project cars 2
_mfcncmf1   = False #codemasters F1
_mfcnac     = False #assetto corsa

#process variables
_p_mfcsvr   = None  #server pid
_p_mfccli   = None  #client pid
_p_mfccmf1  = None  #CM F1 pid
_p_mfcpcars = None  #PCARS2 pid
_p_mfccac   = None  #Assetto Corsa pid

#start MFC server
def mfc_server_start ():
  #os.system("/opt/server-mfc > /tmp/mfc-svr.log 2>&1")
  fret = 0
  global _p_mfcsvr
  fout = open ("/tmp/mfc-svr.log", "wb")
  _p_mfcsvr = pexpect.spawn ('/bin/bash -c "/opt/server-mfc $(cat /opt/server-mfc.default)"', echo=False)
  _p_mfcsvr.logfile = fout
  print ("MFC SVR running with pid {}".format(_p_mfcsvr.pid))
  index = _p_mfcsvr.expect(["#i:ready", pexpect.EOF, pexpect.TIMEOUT], timeout=30)
  if index == 0:
    print ("MFC SVR ready")
  elif index == 1:
    print ("MFC SVR EOF")
    fret = 1
    _p_mfcsvr.sendcontrol('c')
  elif index == 2:
    print ("MFC SVR TMO")
    fret = 2
    _p_mfcsvr.sendcontrol('c')
  #
  #fout.close ()
  return fret

#stop MFC server
def mfc_server_stop ():
  print 'MFC SVR stop'
  global _p_mfcsvr
  try:
    _p_mfcsvr.close (force=True)
    _p_mfcsvr.logfile.close ()
  except:
    print ('MFC SVR process unknown')
  #os.system("killall /opt/server-mfc")
  #time.sleep(10)
  #just making sure
  pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
  for pid in pids:
    try:
      cmd = open (os.path.join ('/proc', pid, 'cmdline'), 'rb').read ()
      #SVR
      if "server-mfc" in cmd:
        print ('SVR found {} pid {}'.format(_mfcsvr, pid))
        os.system ('kill -9 ' + pid)
    except:
      continue

#start MFC client
def mfc_client_start ():
  fret = 0
  global _p_mfccli
  #os.system("/opt/client-mfc > /tmp/mfc-cli.log 2>&1")
  fout = open ("/tmp/mfc-cli.log", "wb")
  _p_mfccli = pexpect.spawn ('/bin/bash -c "/opt/usbxtractor-mfc $(cat /opt/usbxtractor-mfc.default)"', echo=False)
  _p_mfccli.logfile = fout
  print ("MFC CLI running with pid {}".format(_p_mfccli.pid))
  index = _p_mfccli.expect(["#i:ready", pexpect.EOF, pexpect.TIMEOUT], timeout=30)
  if index == 0:
    print ("MFC CLI ready")
  elif index == 1:
    print ("MFC CLI EOF")
    fret = 1
    _p_mfccli.sendcontrol('c')
  elif index == 2:
    print ("MFC CLI TMO")
    fret = 2
    _p_mfccli.sendcontrol('c')
  #
  #fout.close ()
  return fret

#stop MFC client
def mfc_client_stop ():
  print 'MFC CLI stop'
  global _p_mfccli
  try:
    _p_mfccli.close (force=True)
    _p_mfccli.logfile.close ()
  except:
    print ('MFC CLI process unknown')
  #os.system("killall /opt/client-mfc")
  #time.sleep(5)
  #just making sure
  pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
  for pid in pids:
    try:
      cmd = open (os.path.join ('/proc', pid, 'cmdline'), 'rb').read ()
      #CLI
      if "usbxtractor-mfc" in cmd:
        print ('CLI found {} pid {}'.format(_mfccli, pid))
        os.system ('kill -9 ' + pid)
    except:
      continue

#shutdown MFC
def mfc_poweroff ():
  print 'MFC shutdown now'
  os.system("shutdown -h now")

#start MFC native CM F1
def mfc_cmf1_start ():
  print 'MFC CM F1 start'
  fret = 0
  global _p_mfccmf1
  #os.system("/opt/client-mfc > /tmp/mfc-cli.log 2>&1")
  fout = open ("/tmp/mfcc-cmf1d.log", "wb")
  _p_mfccmf1 = pexpect.spawn ('/bin/bash -c "/opt/mfcc-cmf1d $(cat /opt/mfcc-cmf1d.default)"', echo=False)
  _p_mfccmf1.logfile = fout
  print ("MFC CM F1 running with pid {}".format(_p_mfccmf1.pid))
  index = _p_mfccmf1.expect(["#i:ready", pexpect.EOF, pexpect.TIMEOUT], timeout=10)
  if index == 0:
    print ("MFC CM F1 ready")
  elif index == 1:
    print ("MFC CM F1 EOF")
    fret = 1
    _p_mfccmf1.sendcontrol('c')
  elif index == 2:
    print ("MFC CM F1 TMO")
    fret = 2
    _p_mfccmf1.sendcontrol('c')
  #
  #fout.close ()
  return fret

#stop MFC native CM F1
def mfc_cmf1_stop ():
  print 'MFC CM F1 stop'
  global _p_mfccmf1
  try:
    _p_mfccmf1.close (force=True)
    _p_mfccmf1.logfile.close ()
  except:
    print ('MFC CM F1 process unknown')
  #os.system("killall /opt/client-mfc")
  #time.sleep(5)
  #just making sure
  pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
  for pid in pids:
    try:
      cmd = open (os.path.join ('/proc', pid, 'cmdline'), 'rb').read ()
      #SVR
      if "mfcc-cmf1d" in cmd:
        print ('CM F1 found {} pid {}'.format(_mfcncmf1, pid))
        os.system ('kill -9 ' + pid)
    except:
      continue

#start MFC native PCARS
def mfc_pcars_start ():
  print 'MFC PCARS start'
  fret = 0
  global _p_mfcpcars
  #os.system("/opt/client-mfc > /tmp/mfc-cli.log 2>&1")
  fout = open ("/tmp/mfcc-pcars2d.log", "wb")
  _p_mfcpcars = pexpect.spawn ('/bin/bash -c "/opt/mfcc-pcars2d $(cat /opt/mfcc-pcars2d.default)"', echo=False)
  _p_mfcpcars.logfile = fout
  print ("MFC PCARS running with pid {}".format(_p_mfcpcars.pid))
  index = _p_mfcpcars.expect(["#i:ready", pexpect.EOF, pexpect.TIMEOUT], timeout=10)
  if index == 0:
    print ("MFC PCARS ready")
  elif index == 1:
    print ("MFC PCARS EOF")
    fret = 1
    _p_mfcpcars.sendcontrol('c')
  elif index == 2:
    print ("MFC PCARS TMO")
    fret = 2
    _p_mfcpcars.sendcontrol('c')
  #
  #fout.close ()
  return fret

#stop MFC native PCARS
def mfc_pcars_stop ():
  print 'MFC PCARS stop'
  global _p_mfcpcars
  try:
    _p_mfcpcars.close (force=True)
    _p_mfcpcars.logfile.close ()
  except:
    print ('MFC PCARS process unknown')
  #os.system("killall /opt/client-mfc")
  #time.sleep(5)
  #just making sure
  pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
  for pid in pids:
    try:
      cmd = open (os.path.join ('/proc', pid, 'cmdline'), 'rb').read ()
      #SVR
      if "mfcc-pcars2d" in cmd:
        print ('CM PCARS found {} pid {}'.format(_mfcnpcars, pid))
        os.system ('kill -9 ' + pid)
    except:
      continue

#start MFC native ASSETTO CORSA
def mfc_cac_start ():
  print 'MFC AC start'
  fret = 0
  global _p_mfccac
  #os.system("/opt/client-mfc > /tmp/mfc-cli.log 2>&1")
  fout = open ("/tmp/mfcc-acd.log", "wb")
  _p_mfccac = pexpect.spawn ('/bin/bash -c "/opt/mfcc-acd $(cat /opt/mfcc-acd.default)"', echo=False)
  _p_mfccac.logfile = fout
  print ("MFC AC running with pid {}".format(_p_mfccac.pid))
  index = _p_mfccac.expect(["#i:ready", pexpect.EOF, pexpect.TIMEOUT], timeout=10)
  if index == 0:
    print ("MFC AC ready")
  elif index == 1:
    print ("MFC AC EOF")
    fret = 1
    _p_mfccac.sendcontrol('c')
  elif index == 2:
    print ("MFC AC TMO")
    fret = 2
    _p_mfccac.sendcontrol('c')
  #
  #fout.close ()
  return fret

#stop MFC native PCARS
def mfc_cac_stop ():
  print 'MFC AC stop'
  global _p_mfccac
  try:
    _p_mfccac.close (force=True)
    _p_mfccac.logfile.close ()
  except:
    print ('MFC AC process unknown')
  #os.system("killall /opt/client-mfc")
  #time.sleep(5)
  #just making sure
  pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
  for pid in pids:
    try:
      cmd = open (os.path.join ('/proc', pid, 'cmdline'), 'rb').read ()
      #SVR
      if "mfcc-acd" in cmd:
        print ('CM AC found {} pid {}'.format(_mfccac, pid))
        os.system ('kill -9 ' + pid)
    except:
      continue

#check running statuses
def check_status ():
  global _mfcsvr
  global _mfccli
  global _mfcncmf1
  global _mfcnpcars
  _mfcsvr = False
  _mfccli = False
  _mfcnpcars = False
  _mfcncmf1  = False

  pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
  for pid in pids:
    try:
      cmd = open (os.path.join ('/proc', pid, 'cmdline'), 'rb').read ()
      #SVR
      if "server-mfc" in cmd:
        _mfcsvr = True
        print ('SVR found {}'.format(_mfcsvr))
      #CLI
      if "usbxtractor-mfc" in cmd:
        _mfccli = True
        print ('CLI found {}'.format(_mfccli))
      #
      if "mfcc-cmf1d" in cmd:
        _mfcncmf1 = True
        print ('CMF1 found {}'.format(_mfcncmf1))
      #
      if "mfcc-pcars2d" in cmd:
        _mfcnpcars = True
        print ('PCARS found {}'.format(_mfcnpcars))
      #
      if "mfcc-acd" in cmd:
        _mfcnac = True
        print ('AC found {}'.format(_mfcnac))
      #
    except IOError: # proc has already terminated
      continue

# App config
DEBUG = True
app = Flask(__name__)
app.config.from_object(__name__)
app.config['SECRET_KEY'] = '7d441f27d441f27567d441f2b6176a'
 
class ReusableForm(Form):
  #MFC cmf1 client control
  mfcpoweroff = SubmitField(label='SHUTDOWN')
  #MFC server control
  mfcsvron = SubmitField(label='START')
  mfcsvrof = SubmitField(label='STOP')
  #MFC client control
  mfcclion = SubmitField(label='START')
  mfccliof = SubmitField(label='STOP')
  #MFC PCARS client control
  mfcnpcarson = SubmitField(label='START')
  mfcnpcarsof = SubmitField(label='STOP')
  #MFC AC client control
  mfcnacon = SubmitField(label='START')
  mfcnacof = SubmitField(label='STOP')
  #MFC cmf1 client control
  mfcncmf1on = SubmitField(label='START')
  mfcncmf1of = SubmitField(label='STOP')

@app.route("/", methods=['GET', 'POST'])
def cpanel():
  form = ReusableForm (request.form)

  #print form.errors
  if request.method == 'POST':

    if form.validate():
      if form.mfcpoweroff.data:
        flash('Danger: MFC shutting down in 10s')
        poft = threading.Timer (2.0, mfc_poweroff)
        poft.start() # after 30 seconds, "hello, world" will be printed
        return render_template('poweroff.html', mfcpoweroff=True)
      #
      if form.mfcsvron.data:
        rv = mfc_server_start ()
        if rv == 0:
          flash ('MFC Server Ready')
        else:
          flash ('MFC Server Error')
          with open('/tmp/mfc-svr.log') as f:
            for line in f:
              if '#e:' in line.lower():
                flash ('Error - '+line)
      #
      if form.mfcsvrof.data:
        mfc_server_stop ()
        #flash('MFC server stopped')
      #
      if form.mfcclion.data:
        rv = mfc_client_start ()
        #flash ('MFC client started')
        #fout = open ("/tmp/mfc-cli.log", "r")
        #fdata = fout.read().replace('\n', '')
        #flash (fdata)
        if rv == 0:
          flash ('MFC Client Ready')
        else:
          flash ('MFC Client Error')
          with open('/tmp/mfc-cli.log') as f:
            for line in f:
              if '#e:' in line.lower():
                flash ('Error - '+line)
      #
      if form.mfccliof.data:
        mfc_client_stop ()
        #flash('MFC client stopped')
      #
      if form.mfcncmf1on.data:
        rv = mfc_cmf1_start ()
        if rv == 0:
          flash ('MFC CM F1 Ready')
        else:
          flash ('MFC CM F1 Error')
          with open('/tmp/mfcc-cmf1d.log') as f:
            for line in f:
              if '#e:' in line.lower():
                flash ('Error - '+line)
      #
      if form.mfcncmf1of.data:
        mfc_cmf1_stop ()
      #
      if form.mfcnpcarson.data:
        rv = mfc_pcars_start ()
        if rv == 0:
          flash ('MFC PCARS Ready')
        else:
          flash ('MFC PCARS Error')
          with open('/tmp/mfcc-pcars2d.log') as f:
            for line in f:
              if '#e:' in line.lower():
                flash ('Error - '+line)
      #
      if form.mfcnpcarsof.data:
        mfc_pcars_stop ()
      #
      if form.mfcnacon.data:
        rv = mfc_cac_start ()
        if rv == 0:
          flash ('MFC AC Ready')
        else:
          flash ('MFC AC Error')
          with open('/tmp/mfcc-acd.log') as f:
            for line in f:
              if '#e:' in line.lower():
                flash ('Error - '+line)
      #
      if form.mfcnacof.data:
        mfc_cac_stop ()
      #
    else:
      flash('Error: Unable to comply.')
  #
  check_status ()
  #print ('SVR-found {}'.format(_mfcsvr))
  #
  return render_template('cpanel.html', mfcsvr=_mfcsvr, mfccli=_mfccli, mfcnpcars=_mfcnpcars, mfcncmf1=_mfcncmf1, mfcnac=_mfcnac, form=form)

@app.route('/logs/<path:filename>', methods=['GET'])
def download(filename):
  logs = os.path.join ('/tmp/', filename)
  print 'asking for file {}'.format (logs)
  try:
    return send_file (logs)
  except Exception as e:
    return str(e)

if __name__ == "__main__":
  check_status ()
  #app.run()
  app.run (host='0.0.0.0',port='80')
