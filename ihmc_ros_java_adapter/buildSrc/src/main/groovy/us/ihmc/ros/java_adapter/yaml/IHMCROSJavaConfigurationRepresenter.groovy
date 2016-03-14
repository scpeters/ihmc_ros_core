package us.ihmc.ros.java_adapter.yaml

import org.yaml.snakeyaml.nodes.Node
import org.yaml.snakeyaml.nodes.Tag
import org.yaml.snakeyaml.representer.Represent
import org.yaml.snakeyaml.representer.Representer
import us.ihmc.ros.java_adapter.*


class IHMCROSJavaConfigurationRepresenter extends Representer {

    IHMCROSJavaConfigurationRepresenter() {
        this.representers.put(IHMCROSJavaConfiguration.class, new RepresentConfiguration())
        this.representers.put(IHMCROSJavaDependencyHolder.class, new RepresentDependencies())
        this.representers.put(IHMCROSJavaVMConfiguration.class, new RepresentVMConfiguration())
    }

    private class RepresentConfiguration implements Represent {

        @Override
        Node representData(Object o) {
            def config = o as IHMCROSJavaConfiguration
            def depHolder = config.dependencyHolder
            def vmConfig = config.vmConfiguration

            def mapping = new ArrayList<Object>()

            mapping.add(vmConfig)
            mapping.add(depHolder)

            return representSequence(new Tag(IHMCROSJavaConfigurationConstructor.CORE_TAG), mapping, false)
        }
    }

    private class RepresentDependencies implements Represent {
        @Override
        Node representData(Object o) {
            def dependencyHolder = o as IHMCROSJavaDependencyHolder
            return representSequence(new Tag(IHMCROSJavaConfigurationConstructor.DEPENDENCIES_TAG), dependencyHolder.dependencies, false)
        }
    }

    private class RepresentVMConfiguration implements Represent {
        @Override
        Node representData(Object o) {
            def vmConfig = o as IHMCROSJavaVMConfiguration
            def stringMapping = new HashMap<String, String>()

            stringMapping.put("mainMethod", vmConfig.mainMethod)
            stringMapping.put("heapSize", vmConfig.heapSize);

            return representMapping(new Tag(IHMCROSJavaConfigurationConstructor.VM_TAG), stringMapping, false)
        }
    }
}
