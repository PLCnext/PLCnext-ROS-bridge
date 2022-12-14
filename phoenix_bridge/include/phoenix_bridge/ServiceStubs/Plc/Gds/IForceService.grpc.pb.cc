// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: IForceService.proto

#include "IForceService.pb.h"
#include "IForceService.grpc.pb.h"

#include <functional>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/channel_interface.h>
#include <grpcpp/impl/codegen/client_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/message_allocator.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/rpc_service_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/sync_stream.h>
namespace Arp {
namespace Plc {
namespace Gds {
namespace Services {
namespace Grpc {

static const char* IForceService_method_names[] = {
  "/Arp.Plc.Gds.Services.Grpc.IForceService/AddVariable",
  "/Arp.Plc.Gds.Services.Grpc.IForceService/RemoveVariable",
  "/Arp.Plc.Gds.Services.Grpc.IForceService/GetVariables",
  "/Arp.Plc.Gds.Services.Grpc.IForceService/Reset",
  "/Arp.Plc.Gds.Services.Grpc.IForceService/IsForcable",
  "/Arp.Plc.Gds.Services.Grpc.IForceService/IsActive",
};

std::unique_ptr< IForceService::Stub> IForceService::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< IForceService::Stub> stub(new IForceService::Stub(channel));
  return stub;
}

IForceService::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel)
  : channel_(channel), rpcmethod_AddVariable_(IForceService_method_names[0], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_RemoveVariable_(IForceService_method_names[1], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_GetVariables_(IForceService_method_names[2], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_Reset_(IForceService_method_names[3], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_IsForcable_(IForceService_method_names[4], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_IsActive_(IForceService_method_names[5], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status IForceService::Stub::AddVariable(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest& request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest, ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_AddVariable_, context, request, response);
}

void IForceService::Stub::experimental_async::AddVariable(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest, ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_AddVariable_, context, request, response, std::move(f));
}

void IForceService::Stub::experimental_async::AddVariable(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_AddVariable_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse>* IForceService::Stub::PrepareAsyncAddVariableRaw(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse, ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_AddVariable_, context, request);
}

::grpc::ClientAsyncResponseReader< ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse>* IForceService::Stub::AsyncAddVariableRaw(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncAddVariableRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status IForceService::Stub::RemoveVariable(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest& request, ::google::protobuf::Empty* response) {
  return ::grpc::internal::BlockingUnaryCall< ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_RemoveVariable_, context, request, response);
}

void IForceService::Stub::experimental_async::RemoveVariable(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest* request, ::google::protobuf::Empty* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_RemoveVariable_, context, request, response, std::move(f));
}

void IForceService::Stub::experimental_async::RemoveVariable(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest* request, ::google::protobuf::Empty* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_RemoveVariable_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::google::protobuf::Empty>* IForceService::Stub::PrepareAsyncRemoveVariableRaw(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::google::protobuf::Empty, ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_RemoveVariable_, context, request);
}

::grpc::ClientAsyncResponseReader< ::google::protobuf::Empty>* IForceService::Stub::AsyncRemoveVariableRaw(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncRemoveVariableRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status IForceService::Stub::GetVariables(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::google::protobuf::Empty, ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_GetVariables_, context, request, response);
}

void IForceService::Stub::experimental_async::GetVariables(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::google::protobuf::Empty, ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetVariables_, context, request, response, std::move(f));
}

void IForceService::Stub::experimental_async::GetVariables(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetVariables_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse>* IForceService::Stub::PrepareAsyncGetVariablesRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_GetVariables_, context, request);
}

::grpc::ClientAsyncResponseReader< ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse>* IForceService::Stub::AsyncGetVariablesRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncGetVariablesRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status IForceService::Stub::Reset(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::google::protobuf::Empty* response) {
  return ::grpc::internal::BlockingUnaryCall< ::google::protobuf::Empty, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_Reset_, context, request, response);
}

void IForceService::Stub::experimental_async::Reset(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::google::protobuf::Empty* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::google::protobuf::Empty, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_Reset_, context, request, response, std::move(f));
}

void IForceService::Stub::experimental_async::Reset(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::google::protobuf::Empty* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_Reset_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::google::protobuf::Empty>* IForceService::Stub::PrepareAsyncResetRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::google::protobuf::Empty, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_Reset_, context, request);
}

::grpc::ClientAsyncResponseReader< ::google::protobuf::Empty>* IForceService::Stub::AsyncResetRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncResetRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status IForceService::Stub::IsForcable(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest& request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_IsForcable_, context, request, response);
}

void IForceService::Stub::experimental_async::IsForcable(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_IsForcable_, context, request, response, std::move(f));
}

void IForceService::Stub::experimental_async::IsForcable(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_IsForcable_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse>* IForceService::Stub::PrepareAsyncIsForcableRaw(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_IsForcable_, context, request);
}

::grpc::ClientAsyncResponseReader< ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse>* IForceService::Stub::AsyncIsForcableRaw(::grpc::ClientContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncIsForcableRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status IForceService::Stub::IsActive(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::google::protobuf::Empty, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_IsActive_, context, request, response);
}

void IForceService::Stub::experimental_async::IsActive(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::google::protobuf::Empty, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_IsActive_, context, request, response, std::move(f));
}

void IForceService::Stub::experimental_async::IsActive(::grpc::ClientContext* context, const ::google::protobuf::Empty* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_IsActive_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse>* IForceService::Stub::PrepareAsyncIsActiveRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_IsActive_, context, request);
}

::grpc::ClientAsyncResponseReader< ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse>* IForceService::Stub::AsyncIsActiveRaw(::grpc::ClientContext* context, const ::google::protobuf::Empty& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncIsActiveRaw(context, request, cq);
  result->StartCall();
  return result;
}

IForceService::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      IForceService_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< IForceService::Service, ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest, ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](IForceService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest* req,
             ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse* resp) {
               return service->AddVariable(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      IForceService_method_names[1],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< IForceService::Service, ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](IForceService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest* req,
             ::google::protobuf::Empty* resp) {
               return service->RemoveVariable(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      IForceService_method_names[2],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< IForceService::Service, ::google::protobuf::Empty, ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](IForceService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::google::protobuf::Empty* req,
             ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse* resp) {
               return service->GetVariables(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      IForceService_method_names[3],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< IForceService::Service, ::google::protobuf::Empty, ::google::protobuf::Empty, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](IForceService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::google::protobuf::Empty* req,
             ::google::protobuf::Empty* resp) {
               return service->Reset(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      IForceService_method_names[4],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< IForceService::Service, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](IForceService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest* req,
             ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse* resp) {
               return service->IsForcable(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      IForceService_method_names[5],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< IForceService::Service, ::google::protobuf::Empty, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](IForceService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::google::protobuf::Empty* req,
             ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse* resp) {
               return service->IsActive(ctx, req, resp);
             }, this)));
}

IForceService::Service::~Service() {
}

::grpc::Status IForceService::Service::AddVariable(::grpc::ServerContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableRequest* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceAddVariableResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status IForceService::Service::RemoveVariable(::grpc::ServerContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceRemoveVariableRequest* request, ::google::protobuf::Empty* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status IForceService::Service::GetVariables(::grpc::ServerContext* context, const ::google::protobuf::Empty* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceGetVariablesResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status IForceService::Service::Reset(::grpc::ServerContext* context, const ::google::protobuf::Empty* request, ::google::protobuf::Empty* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status IForceService::Service::IsForcable(::grpc::ServerContext* context, const ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableRequest* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsForcableResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status IForceService::Service::IsActive(::grpc::ServerContext* context, const ::google::protobuf::Empty* request, ::Arp::Plc::Gds::Services::Grpc::IForceServiceIsActiveResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace Arp
}  // namespace Plc
}  // namespace Gds
}  // namespace Services
}  // namespace Grpc

