#!/usr/bin/python
import zipfile, os, sys, stat
from urllib2 import urlopen, URLError, HTTPError


try:
    distVersion = "IHMCAtlasAPI-0.2.0-alpha"
    scriptsDir = os.path.dirname(os.path.realpath(__file__))
    destFile = scriptsDir + "/" + distVersion + ".zip"

    if os.path.exists(scriptsDir + "/" + distVersion):
        print "An API instance already exists, please remove " + scriptsDir + "/" + distVersion + " and run the bootstrapper again."
        exit(1)

    distZip = urlopen("https://bitbucket.org/ihmcrobotics/ihmc_msgs/downloads/" + distVersion + ".zip")
    print "Downloading the IHMC Atlas API distribution..."

    with open(destFile, "wb") as local_file:
        local_file.write(distZip.read())

    print "Unzipping and cleaning up..."
    archiveHandle = zipfile.ZipFile(destFile, "r")
    archiveHandle.extractall(scriptsDir)
    os.remove(destFile)

    binDir = scriptsDir + "/" + distVersion + "/bin"
    for script in os.listdir(binDir):
        if not script.endswith("Configurations") and not script.endswith(".bat"):
            os.chmod(binDir + "/" + script, stat.S_IRWXU | stat.S_IRWXG | stat.S_IROTH)

except HTTPError, e:
    print "HTTP Error: ", e.code

except URLError, e:
    print "URL Error: ", e.reason
