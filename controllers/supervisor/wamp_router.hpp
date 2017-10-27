#ifndef H_WAMP_ROUTER_HPP
#define H_WAMP_ROUTER_HPP
#pragma once

// most of the code is taken from bonefish daemon sources.

#include <bonefish/rawsocket/rawsocket_server.hpp>
#include <bonefish/rawsocket/tcp_listener.hpp>

#ifdef BOOST_ASIO_HAS_LOCAL_SOCKETS
#include <bonefish/rawsocket/uds_listener.hpp>
#endif

#include <bonefish/router/wamp_router.hpp>
#include <bonefish/router/wamp_routers.hpp>
#include <bonefish/serialization/json_serializer.hpp>
#include <bonefish/serialization/msgpack_serializer.hpp>
#include <bonefish/serialization/wamp_serializers.hpp>
#include <bonefish/websocket/websocket_server.hpp>

#include <boost/asio/io_service.hpp>

#include <memory>
#include <utility>

#include <iostream>

class wamp_router
{
public:
  wamp_router(std::size_t ws_port, std::size_t rs_port, const std::string& uds_path,
              const std::string& realm)
    : io_service_()
    , work_()
    , routers_(std::make_shared<bonefish::wamp_routers>())
    , serializers_(std::make_shared<bonefish::wamp_serializers>())
    , rs_server_()
    , ws_server_()
    , ws_port_(ws_port)
  {
    routers_->add_router(std::make_shared<bonefish::wamp_router>(io_service_, realm));

    serializers_->add_serializer(std::make_shared<bonefish::json_serializer>());
    serializers_->add_serializer(std::make_shared<bonefish::msgpack_serializer>());

    rs_server_ = std::make_shared<bonefish::rawsocket_server>(routers_, serializers_);
    auto tcp_listener = std::make_shared<bonefish::tcp_listener>(io_service_, boost::asio::ip::address(), rs_port);
    rs_server_->attach_listener(std::static_pointer_cast<bonefish::rawsocket_listener>(tcp_listener));

#ifdef BOOST_ASIO_HAS_LOCAL_SOCKETS
    auto uds_listener = std::make_shared<bonefish::uds_listener>(io_service_, uds_path);
    rs_server_->attach_listener(std::static_pointer_cast<bonefish::rawsocket_listener>(uds_listener));
#endif

    ws_server_ = std::make_shared<bonefish::websocket_server>(io_service_, routers_, serializers_);
  }

  void run()
  {
    work_.reset(new boost::asio::io_service::work(io_service_));
    rs_server_->start();
    ws_server_->start(boost::asio::ip::address(), ws_port_);

    std::cout << "server running" << std::endl;
    io_service_.run();
  }

  void shutdown()
  {
    if(work_) {
      io_service_.post([this]() {
          ws_server_->shutdown();
          rs_server_->shutdown();

          work_.reset();
          io_service_.poll();
          io_service_.stop();
        });
    }
  }

private:
  boost::asio::io_service io_service_;
  std::shared_ptr<boost::asio::io_service::work> work_;

  std::shared_ptr<bonefish::wamp_routers> routers_;
  std::shared_ptr<bonefish::wamp_serializers> serializers_;
  std::shared_ptr<bonefish::rawsocket_server> rs_server_;
  std::shared_ptr<bonefish::websocket_server> ws_server_;

  std::size_t ws_port_;
};

#endif // H_WAMP_ROUTER_HPP
