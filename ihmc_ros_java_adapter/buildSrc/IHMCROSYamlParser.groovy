import groovy.transform.Canonical
import org.yaml.snakeyaml.constructor.AbstractConstruct
import org.yaml.snakeyaml.constructor.Constructor
import org.yaml.snakeyaml.nodes.MappingNode
import org.yaml.snakeyaml.nodes.Node
import org.yaml.snakeyaml.nodes.Tag
import org.yaml.snakeyaml.representer.Represent
import org.yaml.snakeyaml.representer.Representer

@Canonical
class IHMCRosJavaDependencyHolder {
    List<String> dependencies
}

@Canonical
class IHMCRosJavaVMConfiguration {
    String mainMethod
    String heapSize
}

class VMConfigurationRepresenter extends Representer {
    VMConfigurationRepresenter() {
        this.representers.put(IHMCRosJavaVMConfiguration.class, new RepresentVMConfiguration())
    }

    private class RepresentVMConfiguration implements Represent {
        @Override
        Node representData(Object o) {
            def vmConfig = (IHMCRosJavaVMConfiguration) o
            def stringMapping = new HashMap<String, String>()

            stringMapping.put("mainMethod", vmConfig.mainMethod)
            strpingMapping.put("heapSize", vmConfig.heapSize);

            return representMapping(new Tag("!vmConfig"), stringMapping, false)
        }
    }
}

class VMConfigurationConstructor extends Constructor {

    VMConfigurationConstructor() {
        this.yamlConstructors.put(new Tag("!vmConfig"), new ConstructVMConfiguration())
    }

    private class ConstructVMConfiguration extends AbstractConstruct {
        @Override
        Object construct(Node node) {
            Map<String, String> mapping = constructMapping(node as MappingNode) as Map<String, String>;
            def heapSize = mapping.get("heapSize")
            def mainMethod = mapping.get("mainMethod")

            return new IHMCRosJavaVMConfiguration(mainMethod, heapSize)
        }
    }
}