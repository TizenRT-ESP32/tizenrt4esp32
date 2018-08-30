// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: google/protobuf/unittest_import_public.proto

#ifndef PROTOBUF_google_2fprotobuf_2funittest_5fimport_5fpublic_2eproto__INCLUDED
#define PROTOBUF_google_2fprotobuf_2funittest_5fimport_5fpublic_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace protobuf_google_2fprotobuf_2funittest_5fimport_5fpublic_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsPublicImportMessageImpl();
void InitDefaultsPublicImportMessage();
inline void InitDefaults() {
  InitDefaultsPublicImportMessage();
}
}  // namespace protobuf_google_2fprotobuf_2funittest_5fimport_5fpublic_2eproto
namespace protobuf_unittest_import {
class PublicImportMessage;
class PublicImportMessageDefaultTypeInternal;
extern PublicImportMessageDefaultTypeInternal _PublicImportMessage_default_instance_;
}  // namespace protobuf_unittest_import
namespace protobuf_unittest_import {

// ===================================================================

class PublicImportMessage : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:protobuf_unittest_import.PublicImportMessage) */ {
 public:
  PublicImportMessage();
  virtual ~PublicImportMessage();

  PublicImportMessage(const PublicImportMessage& from);

  inline PublicImportMessage& operator=(const PublicImportMessage& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PublicImportMessage(PublicImportMessage&& from) noexcept
    : PublicImportMessage() {
    *this = ::std::move(from);
  }

  inline PublicImportMessage& operator=(PublicImportMessage&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const PublicImportMessage& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PublicImportMessage* internal_default_instance() {
    return reinterpret_cast<const PublicImportMessage*>(
               &_PublicImportMessage_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(PublicImportMessage* other);
  friend void swap(PublicImportMessage& a, PublicImportMessage& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PublicImportMessage* New() const PROTOBUF_FINAL { return New(NULL); }

  PublicImportMessage* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const PublicImportMessage& from);
  void MergeFrom(const PublicImportMessage& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(PublicImportMessage* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional int32 e = 1;
  bool has_e() const;
  void clear_e();
  static const int kEFieldNumber = 1;
  ::google::protobuf::int32 e() const;
  void set_e(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:protobuf_unittest_import.PublicImportMessage)
 private:
  void set_has_e();
  void clear_has_e();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::int32 e_;
  friend struct ::protobuf_google_2fprotobuf_2funittest_5fimport_5fpublic_2eproto::TableStruct;
  friend void ::protobuf_google_2fprotobuf_2funittest_5fimport_5fpublic_2eproto::InitDefaultsPublicImportMessageImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PublicImportMessage

// optional int32 e = 1;
inline bool PublicImportMessage::has_e() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void PublicImportMessage::set_has_e() {
  _has_bits_[0] |= 0x00000001u;
}
inline void PublicImportMessage::clear_has_e() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void PublicImportMessage::clear_e() {
  e_ = 0;
  clear_has_e();
}
inline ::google::protobuf::int32 PublicImportMessage::e() const {
  // @@protoc_insertion_point(field_get:protobuf_unittest_import.PublicImportMessage.e)
  return e_;
}
inline void PublicImportMessage::set_e(::google::protobuf::int32 value) {
  set_has_e();
  e_ = value;
  // @@protoc_insertion_point(field_set:protobuf_unittest_import.PublicImportMessage.e)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace protobuf_unittest_import

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_google_2fprotobuf_2funittest_5fimport_5fpublic_2eproto__INCLUDED
