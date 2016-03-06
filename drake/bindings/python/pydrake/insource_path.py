import os.path

def getDrakePath():
    return os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
