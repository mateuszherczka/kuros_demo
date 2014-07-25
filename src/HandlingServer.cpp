#include "HandlingServer.hpp"

HandlingServer::HandlingServer() {}

HandlingServer::~HandlingServer() {}

void HandlingServer::handleResponse()
{

    cout << "------------------------------------------------" << endl;
    cout << "Response #" << handledCount << endl;

    response.printValues();

    cout << "------------------------------------------------" << endl;

    ++handledCount;
}

void HandlingServer::handleDisconnect()
{
    // we can call startListening() again
    cout << "Server disconnected." << endl;
}

void HandlingServer::loadTrajectories()
{
    // helper for loading
    DataFile dataFile;

    // some trajectories made in matlab
    std::vector <std::string> filenameQueue { "spiral.txt", "fourpoints.txt", "home.txt" };

    // load and enqueue them
    for (auto filename : filenameQueue)
    {
        trajectory_vec trajectory;
        dataFile.loadSpaceDelimited(filename, trajectory);  // store file contents in trajectory vector
        trajectoryQueue.push_back(trajectory);
    }

    /*
    Response modes:

    1   response when done with BCO, about to run a trajectory, done with a trajectory, exiting
    2   mode 1 + stream of responses every N ms while running a trajectory (stream stops when trajectory done)
    3   mode 1 + stream of responses every N ms all the time

    */

    // lets define trajectory parameters (integers)
    info_vec trajInfo { 2       // response mode
                        20      // response stream interval N ms (probably 12ms is the smallest possible)
                        1       // trajectory id, returned by robot when running trajectory
                        1       // 1 = keep running, 0 = exit after finishing trajectory
                        200     // velocity [mm/s], 200 is a comfortable number, max is around 2000
                        20      // distance [mm] when robot is allowed to start approximating a point
                        1       // framecount in trajectory, very important, count the frames in a trajectory_vec
                      };

    // set the framecounts and enqueue
    for (auto trajectory : trajectoryQueue)
    {
        trajInfo[6] = boost::lexical_cast<int>(trajectory.size());
        info
        /*
        HOOPLA what data type is trajectory_vec now again?
        */
    }
}
