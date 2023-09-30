/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: student.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "student.pb-c.h"
void   student__init
                     (Student         *message)
{
  static const Student init_value = STUDENT__INIT;
  *message = init_value;
}
size_t student__get_packed_size
                     (const Student *message)
{
  assert(message->base.descriptor == &student__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t student__pack
                     (const Student *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &student__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t student__pack_to_buffer
                     (const Student *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &student__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
Student *
       student__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Student *)
     protobuf_c_message_unpack (&student__descriptor,
                                allocator, len, data);
}
void   student__free_unpacked
                     (Student *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &student__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCEnumValue student__catch__enum_values_by_number[2] =
{
  { "userError", "STUDENT__CATCH__userError", 0 },
  { "passwordError", "STUDENT__CATCH__passwordError", 1 },
};
static const ProtobufCIntRange student__catch__value_ranges[] = {
{0, 0},{0, 2}
};
static const ProtobufCEnumValueIndex student__catch__enum_values_by_name[2] =
{
  { "passwordError", 1 },
  { "userError", 0 },
};
const ProtobufCEnumDescriptor student__catch__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "Student.catch",
  "catch",
  "Student__Catch",
  "",
  2,
  student__catch__enum_values_by_number,
  2,
  student__catch__enum_values_by_name,
  1,
  student__catch__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
static const ProtobufCFieldDescriptor student__field_descriptors[4] =
{
  {
    "name",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_STRING,
    0,   /* quantifier_offset */
    offsetof(Student, name),
    NULL,
    &protobuf_c_empty_string,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "num",
    2,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(Student, num),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "c_score",
    3,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(Student, c_score),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "ERROR",
    4,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(Student, error),
    &student__catch__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned student__field_indices_by_name[] = {
  3,   /* field[3] = ERROR */
  2,   /* field[2] = c_score */
  0,   /* field[0] = name */
  1,   /* field[1] = num */
};
static const ProtobufCIntRange student__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 4 }
};
const ProtobufCMessageDescriptor student__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "Student",
  "Student",
  "Student",
  "",
  sizeof(Student),
  4,
  student__field_descriptors,
  student__field_indices_by_name,
  1,  student__number_ranges,
  (ProtobufCMessageInit) student__init,
  NULL,NULL,NULL    /* reserved[123] */
};
