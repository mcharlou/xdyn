#include "WebSocketObserverTest.hpp"
#include "WebSocketObserverException.hpp"
#include "WebSocketEndpoint.hpp"
#include "WebSocketServer.hpp"

#include <iostream>

#include <ssc/macros.hpp>
#include TR1INC(memory)

#define WEBSOCKET_ADDRESS "ws://127.0.0.1:9002"

void on_message_string(WSServer* s, websocketpp::connection_hdl hdl, message_ptr msg);
void on_message_string(WSServer* s, websocketpp::connection_hdl hdl, message_ptr msg)
{
    std::cout << "on_message called with hdl: " << hdl.lock().get()
              << " and message: " << msg->get_payload()
              << std::endl;
    if (MESSAGE_SENT != msg->get_payload())
    {
        std::stringstream ss;
        ss << "Message sent does not match payload: MESSAGE_SENT=" << MESSAGE_SENT << " but payload=" << msg->get_payload();
        THROW(__PRETTY_FUNCTION__, WebSocketException, ss.str());
    }
    try
    {
        s->send(hdl, msg->get_payload(), msg->get_opcode());
    }
    catch (const websocketpp::lib::error_code& e)
    {
        std::cout << "Echo failed because: " << e
                  << "(" << e.message() << ")" << std::endl;
    }
}


TEST_F(WebSocketObserverTest, WebSocketEndpoint_should_be_able_to_connect_a_web_socket_server)
{
    TR1(shared_ptr)<WebSocketServer> w(new WebSocketServer(on_message_string));
    {
        WebSocketEndpoint endpoint(WEBSOCKET_ADDRESS);
        ASSERT_TRUE(endpoint.good());
    }
}

TEST_F(WebSocketObserverTest, WebSocketEndpoint_should_be_able_to_send_a_string)
{
    TR1(shared_ptr)<WebSocketServer> w(new WebSocketServer(on_message_string));
    {
        WebSocketEndpoint endpoint(WEBSOCKET_ADDRESS);
        endpoint.send(MESSAGE_SENT);
    }
}

template<typename T>
std::string convert_vector_to_string(const std::vector<T>& v);
template<typename T>
typename std::vector<T> convert_string_to_vector(const std::string& s);

template<typename T>
std::string convert_vector_to_string(const std::vector<T>& v)
{
    const size_t ss = v.size()*sizeof(T);
    std::string s(ss, 0);
    memcpy((void*)s.c_str(),v.data(),ss);
    return s;
}

template<typename T>
typename std::vector<T> convert_string_to_vector(const std::string& s)
{
    std::vector<T> res;
    const size_t n = s.size()/sizeof(T);
    res.resize(n);
    for (size_t i=0;i<n;++i)
    {
        res[i]=*reinterpret_cast<const double*>(s.c_str() + sizeof(T)*i);
    }
    return res;
}

void on_message_vector(WSServer* s, websocketpp::connection_hdl hdl, message_ptr msg);
void on_message_vector(WSServer* s, websocketpp::connection_hdl hdl, message_ptr msg)
{
    ASSERT_EQ(websocketpp::frame::opcode::binary, msg->get_opcode());
    const std::string payload = msg->get_payload();
    std::vector<double> vv = convert_string_to_vector<double>(payload);
    ASSERT_EQ(3,payload.size()/8);
    ASSERT_EQ(3,vv.size());
    ASSERT_EQ(1.0,vv[0]);
    ASSERT_EQ(2.0,vv[1]);
    ASSERT_EQ(3.0,vv[2]);
    s->send(hdl, msg->get_payload(), msg->get_opcode());
}

TEST_F(WebSocketObserverTest, WebSocketEndpoint_should_be_able_to_send_a_vector_doubles)
{
    TR1(shared_ptr)<WebSocketServer> w(new WebSocketServer(on_message_vector));
    {
        std::vector<double> v(3,0.0);
        v[0]=1.0;v[1]=2.0;v[2]=3.0;
        WebSocketEndpoint endpoint(WEBSOCKET_ADDRESS);
        endpoint.send(v);
    }
}

TEST_F(WebSocketObserverTest, should_be_able_to_convert_vector_of_doubles_to_string_and_back)
{
    const size_t n=3;
    std::vector<double> v(n,0.0);
    v[0]=1.0;v[1]=2.0;v[2]=3.0;
    const std::string s = convert_vector_to_string(v);
    ASSERT_EQ(3*8,s.size());
    std::vector<double> v2=convert_string_to_vector<double>(s);
    ASSERT_EQ(n,v2.size());
    for (size_t i=0;i<n;++i)
    {
        ASSERT_EQ(v[i],v2[i]);
    }
}
