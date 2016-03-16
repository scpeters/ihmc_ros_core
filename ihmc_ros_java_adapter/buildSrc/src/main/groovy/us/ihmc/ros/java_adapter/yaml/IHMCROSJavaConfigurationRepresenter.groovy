package us.ihmc.ros.java_adapter.yaml

import org.yaml.snakeyaml.nodes.Node
import org.yaml.snakeyaml.nodes.Tag
import org.yaml.snakeyaml.representer.Represent
import org.yaml.snakeyaml.representer.Representer
import us.ihmc.ros.java_adapter.*


class IHMCROSJavaConfigurationRepresenter extends Representer {

    IHMCROSJavaConfigurationRepresenter() {
        this.representers.put(IHMCROSJavaConfiguration.class, new RepresentConfiguration())
    }

    private class RepresentConfiguration implements Represent {

        @Override
        Node representData(Object o) {
            def config = o as IHMCROSJavaConfiguration
            def depHolder = config.dependencyHolder
            def vmConfig = config.vmConfiguration

            def mapping = new HashMap<String, Object>()

            def vmArgsMapping = new HashMap<String, Object>()

            vmArgsMapping.put("mainMethod", vmConfig.mainMethod)
            vmArgsMapping.put("heapSize", vmConfig.heapSize)

            mapping.put(IHMCROSJavaConfigurationConstructor.VM_TAG, vmArgsMapping)
            mapping.put(IHMCROSJavaConfigurationConstructor.DEPENDENCIES_TAG, depHolder.dependencies)
            mapping.put(IHMCROSJavaConfigurationConstructor.ARGS_TAG, config.programArguments)

            return representMapping(new Tag(IHMCROSJavaConfigurationConstructor.CORE_TAG), mapping, false)
        }
    }
}
