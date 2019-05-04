
# this shouldn't be here
SECRET_KEY = '7d441f27d441f27567d441f2b6176a'

PROCESSES_CONFIG = [
    {
        'ref': 'mfcsvr',
        'name': 'MFC SVR',
        'label': 'MFC server',
        'desc': 'platform mover server',
        'logfile': '/tmp/mfc-svr.log',
        'bin': 'server-mfc',
    },
    {
        'ref': 'mfccli',
        'name': 'MFC CLI',
        'label': 'MFC client',
        'desc': 'wheel client extractor',
        'logfile': '/tmp/mfc-cli.log',
        'bin': 'usbxtractor-mfc',
    },
    {
        'ref': 'mfccmf1',
        'name': 'MFC CM F1',
        'label': 'CM F1 native client',
        'desc': 'codemasters F1',
        'bin': 'mfcc-cmf1d',
        'expect_timeout': 10,
    },
    {
        'ref': 'mfcpcars',
        'name': 'MFC PCARS',
        'label': 'PCARS native client',
        'desc': 'project cars 2',
        'bin': 'mfcc-pcars2d',
        'expect_timeout': 10,
    },
    {
        'ref': 'mfcnac',
        'name': 'MFC AC',
        'label': 'Assetto Corsa native client',
        'desc': 'assetto corsa',
        'bin': 'mfcc-acd',
        #'logfile': '<some other logfile>',
        #'runcmd': '<some other command>',
        'expect_timeout': 10,
    }
]