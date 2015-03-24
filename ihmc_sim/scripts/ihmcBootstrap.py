#!/usr/bin/python
import tarfile, os, sys, stat, rospkg
from urllib2 import urlopen, URLError, HTTPError


try:
    rospack = rospkg.RosPack()
    distVersion = "IHMCAtlasAPI-0.3.0-alpha"
    ihmcSimDir = rospack.get_path('ihmc_sim')
    distVersionDir = ihmcSimDir + "/" + distVersion
    destFile = distVersionDir + ".tar"

    if os.path.exists(distVersionDir):
        print "An API instance already exists, please remove " + distVersionDir + " and run the bootstrapper again."
        exit(1)

    distTar = urlopen("https://bitbucket.org/ihmcrobotics/ihmc_ros/downloads/" + distVersion + ".tar")
    print "Downloading the IHMC Atlas API distribution..."

    with open(destFile, "wb") as local_file:
        local_file.write(distTar.read())

    print "Untarring and cleaning up..."
    archiveHandle = tarfile.TarFile(destFile, "r")
    archiveHandle.extractall(ihmcSimDir)
    os.remove(destFile)

    binDir = distVersionDir + "/bin"
    for script in os.listdir(binDir):
        if not script.endswith("Configurations") and not script.endswith(".bat"):
            os.chmod(binDir + "/" + script, stat.S_IRWXU | stat.S_IRWXG | stat.S_IROTH)

except HTTPError, e:
    print "HTTP Error: ", e.code

except URLError, e:
    print "URL Error: ", e.reason
