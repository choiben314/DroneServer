// System Includes
#include <iostream>
#include <condition_variable>
#include <iostream>
#include <mutex>

// C Includes
#include <signal.h>

// TCP Includes
#include <tacopie/tacopie>

using namespace std;

#ifdef _WIN32
#include <Winsock2.h>
#endif /* _WIN32 */

condition_variable cv;

void signint_handler(int) {
    cv.notify_all();
}

void on_new_message(const shared_ptr<tacopie::tcp_client>& client, const tacopie::tcp_client::read_result& res) {
    if (res.success) {
        vector<char> msg_buffer = res.buffer;
        string msg_string(msg_buffer.begin(), msg_buffer.end());

        string hostname = client->get_host();
        uint32_t port = client->get_port();

        cout << "Received a new message from " << hostname << " on port " << to_string(port) << ": " << msg_string << endl;
        //string msg_return = "Server received the following message: " + msg_string;
        string msg_return = msg_string;

        vector<char> return_vec(msg_return.begin(), msg_return.end());

        client->async_write({ return_vec, nullptr });
        client->async_read({ 1024, bind(&on_new_message, client, placeholders::_1) });
    }
    else {
        cout << "Client disconnected" << endl;
        client->disconnect();
    }
}

int main(int argc, char* argv[]) {
#ifdef _WIN32
    //! Windows netword DLL init
    WORD version = MAKEWORD(2, 2);
    WSADATA data;

    if (WSAStartup(version, &data) != 0) {
        cerr << "WSAStartup() failure" << endl;
        return -1;
    }
#endif /* _WIN32 */

    tacopie::tcp_server s;

    // 0.0.0.0 listens on all interfaces (127.0.0.1 only accessible from localhost)
    s.start("0.0.0.0", 3001, [](const shared_ptr<tacopie::tcp_client>& client) -> bool {
        string hostname = client->get_host();
        uint32_t port = client->get_port();

        cout << "New client from " + hostname + " on port " + to_string(port) << endl;
        client->async_read({ 1024, bind(&on_new_message, client, placeholders::_1) });
        return true;
        });

    signal(SIGINT, &signint_handler);

    mutex mtx;
    unique_lock<mutex> lock(mtx);
    cv.wait(lock);

#ifdef _WIN32
    WSACleanup();
#endif /* _WIN32 */

    return 0;
}