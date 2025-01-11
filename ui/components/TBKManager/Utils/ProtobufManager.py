import sys
import types
from google.protobuf.internal import builder
from google.protobuf import descriptor_pool, descriptor_pb2


class ProtobufManager:
    def __init__(self):
        self.all_models = {}
        self.remote_manager = None

    def __getattr__(self, name):
        # 如果属性在 all_models 中存在
        for k, v in self.all_models.items():
            if name in v.__dict__:
                return getattr(v, name)
        raise AttributeError(f"Can not find msg_type: {name}.")

    def load_from_file(self, filename):
        pass

    def load_from_descriptor(self, descriptor):
        # 解析 descriptor
        file_desc_proto = descriptor_pb2.FileDescriptorProto.FromString(descriptor)
        name = file_desc_proto.name
        # dependency_list = file_desc_proto.dependency_list
        self.all_models[name] = types.ModuleType(name)
        sys.modules[name] = self.all_models[name]

        # 加载 descriptor
        DESCRIPTOR = descriptor_pool.Default().AddSerializedFile(descriptor)
        package_name = DESCRIPTOR.package
        builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, package_name, self.all_models[name].__dict__)

    # def load_from_

    def load_msg_type_from_remote(self):
        pass

    def check_dependent_msg_types(self, descriptor):
        file_desc_proto = descriptor_pb2.FileDescriptorProto.FromString(descriptor)
        deps = set()
        for msg in file_desc_proto.message_type:
            for field in msg.field:
                # 获取类型名称
                # type_name = descriptor_pb2.FieldDescriptorProto.Type.Name(field.type)
                if field.type == descriptor_pb2.FieldDescriptorProto.Type.TYPE_MESSAGE:
                    # print(field.name, field.type_name)
                    deps.add(field.type_name[1:])
        return deps
