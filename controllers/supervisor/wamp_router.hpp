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
#include <boost/random/random_device.hpp>

#include <memory>
#include <utility>

#include <iostream>

#ifdef BOOST_ASIO_HAS_LOCAL_SOCKETS
namespace /* anonymous */ {

  std::string random_string(std::size_t len)
  {
    constexpr const char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    constexpr std::size_t alphanum_size = sizeof(alphanum) - 1; // -1 to exclude terminating null

    boost::random_device rd{};
    std::default_random_engine re{rd()};
    std::uniform_int_distribution<std::size_t> uid(0, alphanum_size - 1);

    std::string ret;
    std::generate_n(std::back_inserter(ret), len, [&]() { return alphanum[uid(re)]; });
    return ret;
  }

} // namespace /* anonymous */
#endif

class wamp_router
{
public:
  wamp_router(const std::string& realm)
    : io_service_()
    , work_()
    , routers_(std::make_shared<bonefish::wamp_routers>())
    , serializers_(std::make_shared<bonefish::wamp_serializers>())
    , rs_server_()
    , ws_server_()
  {
    routers_->add_router(std::make_shared<bonefish::wamp_router>(io_service_, realm));

    serializers_->add_serializer(std::make_shared<bonefish::json_serializer>());
    serializers_->add_serializer(std::make_shared<bonefish::msgpack_serializer>());

    while (true) { //a while loop to ensure finding a free port/path
      rs_port_ = find_free_port();

      rs_server_ = std::make_shared<bonefish::rawsocket_server>(routers_, serializers_);
      auto tcp_listener = std::make_shared<bonefish::tcp_listener>(io_service_, boost::asio::ip::address(), rs_port_);
      rs_server_->attach_listener(std::static_pointer_cast<bonefish::rawsocket_listener>(tcp_listener));

#ifdef BOOST_ASIO_HAS_LOCAL_SOCKETS
      uds_path_ = "/tmp/aiwc-" + random_string(10) + ".sock";
      auto uds_listener = std::make_shared<bonefish::uds_listener>(io_service_, uds_path_);
      rs_server_->attach_listener(std::static_pointer_cast<bonefish::rawsocket_listener>(uds_listener));
#else
      (void)uds_path_;
#endif

      try {
        rs_server_->start();
        break; //if listening succeeds, leave the loop and proceed
      }
      catch (const boost::system::system_error& err) {
        if (err.code() != make_error_code(boost::asio::error::address_in_use))
          throw std::runtime_error(err.what());
      }
    }

    ws_server_ = std::make_shared<bonefish::websocket_server>(io_service_, routers_, serializers_);
    ws_server_->start(boost::asio::ip::address(), 0); //find an available port and use it
  }

  void run()
  {
    work_.reset(new boost::asio::io_service::work(io_service_));

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

#ifdef BOOST_ASIO_HAS_LOCAL_SOCKETS
    unlink(uds_path_.c_str());
#endif
  }

  std::size_t get_rs_port()
  {
    return rs_port_;
  }

  std::string get_uds_path()
  {
    return uds_path_;
  }

private:

  std::size_t find_free_port() const
  {
    boost::asio::io_service service;
    boost::asio::ip::tcp::acceptor acceptor(service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 0));
    return acceptor.local_endpoint().port();
  }

  boost::asio::io_service io_service_;
  std::shared_ptr<boost::asio::io_service::work> work_;

  std::shared_ptr<bonefish::wamp_routers> routers_;
  std::shared_ptr<bonefish::wamp_serializers> serializers_;
  std::shared_ptr<bonefish::rawsocket_server> rs_server_;
  std::shared_ptr<bonefish::websocket_server> ws_server_;

  std::size_t rs_port_;
  std::string uds_path_;
};

#endif // H_WAMP_ROUTER_HPP
