package us.ihmc.ros.java_adapter

import groovy.transform.Canonical

@Canonical
class IHMCROSJavaConfiguration {
    IHMCROSJavaDependencyHolder dependencyHolder
    IHMCROSJavaVMConfiguration vmConfiguration
    ArrayList<String> programArguments
}
