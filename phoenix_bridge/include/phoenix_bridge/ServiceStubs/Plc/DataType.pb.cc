// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: DataType.proto

#include "DataType.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
namespace Arp {
namespace Plc {
namespace Grpc {
}  // namespace Grpc
}  // namespace Plc
}  // namespace Arp
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_DataType_2eproto = nullptr;
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_DataType_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_DataType_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_DataType_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_DataType_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\016DataType.proto\022\014Arp.Plc.Grpc*\373\005\n\010DataT"
  "ype\022\013\n\007DT_None\020\000\022\013\n\007DT_Void\020\001\022\n\n\006DT_Bit\020"
  "\002\022\016\n\nDT_Boolean\020\003\022\014\n\010DT_UInt8\020\004\022\013\n\007DT_In"
  "t8\020\005\022\014\n\010DT_Char8\020\006\022\r\n\tDT_Char16\020\007\022\r\n\tDT_"
  "UInt16\020\010\022\014\n\010DT_Int16\020\t\022\r\n\tDT_UInt32\020\n\022\014\n"
  "\010DT_Int32\020\013\022\r\n\tDT_UInt64\020\014\022\014\n\010DT_Int64\020\r"
  "\022\016\n\nDT_Float32\020\016\022\016\n\nDT_Float64\020\017\022\020\n\014DT_P"
  "rimitive\020 \022\017\n\013DT_DateTime\020!\022\016\n\nDT_IecTim"
  "e\020\"\022\020\n\014DT_IecTime64\020#\022\016\n\nDT_IecDate\020$\022\020\n"
  "\014DT_IecDate64\020%\022\022\n\016DT_IecDateTime\020&\022\024\n\020D"
  "T_IecDateTime64\020\'\022\023\n\017DT_IecTimeOfDay\020(\022\025"
  "\n\021DT_IecTimeOfDay64\020)\022\023\n\017DT_StaticString"
  "\020*\022\020\n\014DT_IecString\020+\022\020\n\014DT_ClrString\020,\022\r"
  "\n\tDT_String\020-\022\021\n\rDT_Elementary\020@\022\023\n\017DT_A"
  "rrayElement\020A\022\r\n\tDT_Struct\020B\022\014\n\010DT_Class"
  "\020C\022\024\n\020DT_FunctionBlock\020D\022\020\n\014DT_Subsystem"
  "\020E\022\016\n\nDT_Program\020F\022\020\n\014DT_Component\020G\022\016\n\n"
  "DT_Library\020H\022\017\n\nDT_Complex\020\376\001\022\017\n\nDT_Poin"
  "ter\020\200\004\022\r\n\010DT_Array\020\200\010\022\014\n\007DT_Enum\020\200\020\022\021\n\014D"
  "T_Reference\020\200 \022\024\n\017DT_BaseTypeMask\020\377\001b\006pr"
  "oto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_DataType_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_DataType_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_DataType_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_DataType_2eproto = {
  false, false, descriptor_table_protodef_DataType_2eproto, "DataType.proto", 804,
  &descriptor_table_DataType_2eproto_once, descriptor_table_DataType_2eproto_sccs, descriptor_table_DataType_2eproto_deps, 0, 0,
  schemas, file_default_instances, TableStruct_DataType_2eproto::offsets,
  file_level_metadata_DataType_2eproto, 0, file_level_enum_descriptors_DataType_2eproto, file_level_service_descriptors_DataType_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_DataType_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_DataType_2eproto)), true);
namespace Arp {
namespace Plc {
namespace Grpc {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* DataType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_DataType_2eproto);
  return file_level_enum_descriptors_DataType_2eproto[0];
}
bool DataType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 32:
    case 33:
    case 34:
    case 35:
    case 36:
    case 37:
    case 38:
    case 39:
    case 40:
    case 41:
    case 42:
    case 43:
    case 44:
    case 45:
    case 64:
    case 65:
    case 66:
    case 67:
    case 68:
    case 69:
    case 70:
    case 71:
    case 72:
    case 254:
    case 255:
    case 512:
    case 1024:
    case 2048:
    case 4096:
      return true;
    default:
      return false;
  }
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace Grpc
}  // namespace Plc
}  // namespace Arp
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
