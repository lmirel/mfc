
# this shouldn't be here
SECRET_KEY = '7d441f27d441f27567d441f2b6176a'

PROCESSES_CONFIG = [
    {
        'ref': 'mfc-server',
        'name': 'MFC SVR',
        'label': 'MFC server',
        'desc': 'platform motion control server',
        'bin': 'mfc-server',
    },
    {
        'ref': 'mfc-xtract',
        'name': 'MFC Xtractor client',
        'label': 'MFC Xtractor client',
        'desc': 'mfc telemetry extractor',
        'bin': 'mfc-xtract',
    },
    {
        'ref': 'usb-xtract',
        'name': 'USB Xtractor',
        'label': 'USB Xtractor',
        'desc': 'HID controller proxy',
        'bin': 'usb-xtract',
    },
    {
        'ref': 'mfc-cli-cm-f1',
        'name': 'MFC CM F1',
        'label': 'CM F1 native client',
        'desc': 'codemasters F1',
        'bin': 'mfc-cli-cmf1d',
        'expect_timeout': 10,
    },
    {
        'ref': 'mfc-cli-pcars2',
        'name': 'MFC PCARS',
        'label': 'PCARS native client',
        'desc': 'project cars 2',
        'bin': 'mfc-cli-pcars2d',
        'expect_timeout': 10,
    },
    {
        'ref': 'mfc-cli-nac',
        'name': 'MFC AC',
        'label': 'Assetto Corsa native client',
        'desc': 'assetto corsa',
        'bin': 'mfc-cli-acd',
        #'logfile': '<some other logfile>',
        #'runcmd': '<some other command>',
        'expect_timeout': 10,
    }
]