#!/usr/bin/python
import tarfile, os, sys, stat
from urllib2 import urlopen, URLError, HTTPError


try:
    distVersion = "IHMCAtlasAPI-0.2.1-alpha"
    scriptsDir = os.path.dirname(os.path.realpath(__file__))
    destFile = scriptsDir + "/" + distVersion + ".tar"

    if os.path.exists(scriptsDir + "/" + distVersion):
        print "An API instance already exists, please remove " + scriptsDir + "/" + distVersion + " and run the bootstrapper again."
        exit(1)

    distTar = urlopen("https://bitbucket.org/ihmcrobotics/ihmc_msgs/downloads/" + distVersion + ".tar")
    print "Downloading the IHMC Atlas API distribution..."

    with open(destFile, "wb") as local_file:
        local_file.write(distTar.read())

    print "Untarring and cleaning up..."
    archiveHandle = tarfile.TarFile(destFile, "r")
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
