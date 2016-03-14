package us.ihmc.ros.java_adapter.yaml

import org.yaml.snakeyaml.constructor.AbstractConstruct
import org.yaml.snakeyaml.constructor.Constructor
import org.yaml.snakeyaml.nodes.MappingNode
import org.yaml.snakeyaml.nodes.Node
import org.yaml.snakeyaml.nodes.SequenceNode
import org.yaml.snakeyaml.nodes.Tag
import us.ihmc.ros.java_adapter.*


class IHMCROSJavaConfigurationConstructor extends Constructor {
    public static final String CORE_TAG = "!ihmc_ros_java_adapter"
    public static final String DEPENDENCIES_TAG = "!dependencies"
    public static final String VM_TAG = "!vmConfig"

    public IHMCROSJavaConfigurationConstructor() {
        this.yamlConstructors.put(new Tag(CORE_TAG), new ConstructConfiguration())
        this.yamlConstructors.put(new Tag(VM_TAG), new ConstructVMConfiguration())
        this.yamlConstructors.put(new Tag(DEPENDENCIES_TAG), new ConstructDependencies())
    }

    private class ConstructConfiguration extends AbstractConstruct {

        @Override
        Object construct(Node node) {
            def configMap = constructSequence(node as SequenceNode)
            IHMCROSJavaConfiguration yamlConfiguration = new IHMCROSJavaConfiguration();

            configMap.forEach({ yamlObject ->
                if (yamlObject instanceof IHMCROSJavaVMConfiguration)
                    yamlConfiguration.vmConfiguration = yamlObject as IHMCROSJavaVMConfiguration
                else if (yamlObject instanceof IHMCROSJavaDependencyHolder)
                    yamlConfiguration.dependencyHolder = yamlObject as IHMCROSJavaDependencyHolder
            })

            return yamlConfiguration
        }
    }

    private class ConstructVMConfiguration extends AbstractConstruct {
        @Override
        Object construct(Node node) {
            Map<String, String> mapping = constructMapping(node as MappingNode) as Map<String, String>;
            def heapSize = mapping.get("heapSize")
            def mainMethod = mapping.get("mainMethod")

            return new IHMCROSJavaVMConfiguration(mainMethod, heapSize)
        }
    }

    private class ConstructDependencies extends AbstractConstruct {
        @Override
        Object construct(Node node) {
            def dependenciesList = constructSequence(node as SequenceNode) as List<String>
            return new IHMCROSJavaDependencyHolder(dependenciesList)
        }
    }
}
