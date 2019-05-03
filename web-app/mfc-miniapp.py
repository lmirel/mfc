#!/usr/bin/env python

import os
import threading
import pexpect
from flask import Flask, render_template, flash, request, send_file
from collections import OrderedDict

from pexpect import ExceptionPexpect
from wtforms import Form, SubmitField

app = Flask(__name__)
app.config.from_object('settings')


# jinja2 filter
@app.template_filter('basename')
def basename(path):
    return os.path.basename(path)


class ServiceEntity(object):
    default_runcmd = '/bin/bash -c "/opt/{0} $(cat /opt/{0}.default)"'
    default_logfile = '/tmp/{0}.log'

    expected_responses = ["#i:ready", pexpect.EOF, pexpect.TIMEOUT]

    def __init__(self, ref, name, bin, logfile=None, runcmd=None, expect_timeout=30, label='Generic', *args, **kwargs):
        self.name = name
        self.ref = ref
        self.bin = bin
        self.label = label
        self.expect_timeout = expect_timeout
        self.status = False
        self.logfile = logfile if logfile else self.default_logfile.format(self.bin)
        self.runcmd = runcmd if runcmd else self.default_runcmd.format(self.bin)

        self.btn_on = '{}on'.format(self.ref)
        self.btn_off = '{}off'.format(self.ref)

    def start(self):
        print("{} start".format(self.name))

        self.log_fp = open(self.logfile, "wb")

        try:
            self.proc_ref = pexpect.spawn(self.runcmd, echo=False)
        except Exception as err:
            self.log_fp.close()
            raise
        self.proc_ref.logfile = self.log_fp

        # better use logger
        print("{} running with pid {}".format(self.name, self.proc_ref.pid))

        fret = self.proc_ref.expect(self.expected_responses, timeout=self.expect_timeout)

        if fret == 0:
            print("{} ready".format(self.name))
        elif fret == 1:
            print("{} EOF".format(self.name))
            self.proc_ref.sendcontrol('c')
        elif fret == 2:
            print("{} TMO".format(self.name))
            self.proc_ref.sendcontrol('c')

        return fret

    def stop(self):
        print("{} start".format(self.name))
        try:
            self.proc_ref.close(force=True)
        except:
            print('{} process unknown'.format(self.name))
        finally:
            self.proc_ref.logfile.close()

        # maybe use psutil https://psutil.readthedocs.io/en/latest/#processes
        pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
        for pid in pids:
            try:
                with open(os.path.join('/proc', pid, 'cmdline'), 'rb') as fp:
                    cmd = fp.read()
                    if self.bin in cmd:
                        print ('{} found {} pid {}'.format(self.name, self.status, pid))
                        os.system('kill -9 ' + pid)
            except:
                continue


processes = OrderedDict()

for proc_data in app.config['PROCESSES_CONFIG']:
    processes[proc_data['ref']] = ServiceEntity(**proc_data)


# shutdown MFC
def mfc_poweroff():
    print('MFC shutdown now')
    os.system("shutdown -h now")


# check running statuses
def check_status():
    pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
    for pid in pids:
        try:
            cmd = open(os.path.join('/proc', pid, 'cmdline'), 'rb').read()
            for proc in processes.values():
                if proc.bin in cmd:
                    proc.status = True
                    print('{} found {}'.format(proc.name, proc.status))
        except IOError:  # proc has already terminated
            continue


class ReusableForm(Form):
    # MFC cmf1 client control
    mfcpoweroff = SubmitField(label='SHUTDOWN')


btn_mappings = {}

# setting process specific fields dinamically
# and button bindings with method refs
for item in processes.values():

    setattr(ReusableForm, item.btn_on, SubmitField(label='START'))
    btn_mappings[item.btn_on] = (item, True,)

    setattr(ReusableForm, item.btn_off, SubmitField(label='STOP'))
    btn_mappings[item.btn_off] = (item, True,)


@app.route("/", methods=['GET', 'POST'])
def cpanel():
    form = ReusableForm(request.form)

    if request.method == 'POST':

        if form.validate():

            if form.mfcpoweroff.data:
                flash('Danger: MFC shutting down in 10s')
                poft = threading.Timer(2.0, mfc_poweroff)
                poft.start()  # after 30 seconds, "hello, world" will be printed
                return render_template('poweroff.html', mfcpoweroff=True)

            proc_obj, is_start = None, None
            for key, val in form.data.items():
                if val and key in btn_mappings:
                    proc_obj, is_start = btn_mappings.get(key)  # tuple unpacking from the mapping
                    break

            if proc_obj:
                if is_start:
                    try:
                        rv = proc_obj.start()
                    except ExceptionPexpect as err:
                        rv = None  # none to skip the following steps
                        flash("{} Error - '{}' {}".format(proc_obj.name, proc_obj.runcmd, err))

                    if rv == 0:
                        flash('{} Ready'.format(proc_obj.name))
                    elif rv:
                        flash('{} Error'.format(proc_obj.name))
                        with open(proc_obj.logfile) as fp:
                            for line in fp:
                                if '#e:' in line.lower():
                                    flash('Error - ' + line)
                else:
                    proc_obj.stop()
        else:
            flash('Error: Unable to comply.')

    check_status()
    return render_template('cpanel.html', processes=processes.values(), form=form)


@app.route('/logs/<path:filename>', methods=['GET'])
def download(filename):
    logs = os.path.join('/tmp/', filename)
    print('Asking for file {}.'.format(logs))
    try:
        return send_file(logs)
    except Exception as e:
        return str(e)


if __name__ == "__main__":
    check_status()
    app.run(host='127.0.0.1', port=80)
