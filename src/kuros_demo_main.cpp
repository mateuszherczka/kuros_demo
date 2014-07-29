#include <iostream>

#include <HandlingServer.hpp>   // implemented by user

int main()
{

    // start server
    HandlingServer aserver;     // loads config file on creation

    // connect
    aserver.startListening();   // blocks until connection

    // from now on the HandlingServer deals with the robot.

    // idle
    while( aserver.isConnected() ) {

        boost::this_thread::sleep( boost::posix_time::milliseconds(100));   // idle
    }

    cout << "Hello world!" << endl;
    return 0;
}
