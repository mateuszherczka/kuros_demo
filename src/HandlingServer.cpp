#include "HandlingServer.hpp"

HandlingServer::HandlingServer() {}

HandlingServer::~HandlingServer() {}

void HandlingServer::handleResponse()
{
    /*
    Response info vector:

    index   message
    ---------------
    0       Status
    1       Trajectory Id
    2       Tick (sent time)

    */

    robotStatus = response.info[0];

    switch (robotStatus)
    {
    case 0: // not connected or exiting
        cout << "Robot offline." << endl;
        break;

    case 1: // BCO completed, ready

        loadTrajectories(); // load trajectories

        sendNextTrajectory();

        break;

    case 2: // Received trajectory, about to move

        if (!nowCapturing)
        {
            startCapturing();
        }


    case 3: // Executing trajectory, moving

        // continue capturing frames from robot
        capturedFrames.push_back(response.frame);

    case 4: // Finished trajectory, ready for next

        if (nowCapturing)
        {
            finishCapturing();
        }

        // send next available trajectory

        sendNextTrajectory();

        break;
    }

    // print the non-stream responses
    if (robotStatus != 3)
    {
        cout << "------------------------------------------------" << endl;
        cout << "Response #" << handledCount << endl;

        response.printValues();

        cout << "------------------------------------------------" << endl;
    }


    ++handledCount;
}

void HandlingServer::handleDisconnect()
{
    // we can call startListening() again
    cout << "Server disconnected." << endl;
}

void HandlingServer::sendNextTrajectory()
{
    if (!infoQueue.empty() && !trajectoryQueue.empty())
        {
            sendTrajectory(infoQueue.front(),trajectoryQueue.front());    // send the first one
            infoQueue.pop();
            trajectoryQueue.pop();
        }
}

void HandlingServer::startCapturing()
{
    // start capturing frames from robot
    capturedFrames.clear();
    nowCapturing = true;
    capturedFrames.push_back(response.frame);
}

void HandlingServer::finishCapturing()
{
    // end capturing frames from robot
    capturedFrames.push_back(response.frame);
    nowCapturing = false;

    // write captured to file
    cout << "----------------------------------" << endl;
    cout << "Saving trajectory id " << response.info[1] << endl;
    std::string captureName = (boost::format("captured_%1%.txt") %response.info[1]).str();
    cout << "Filename:  " << captureName << endl;
    cout << "----------------------------------" << endl;
    try
    {
        std::ofstream captureFile;
        captureFile.open( captureName );

        for (auto frame : capturedFrames)
        {
            for (auto val : frame)
            {
                captureFile << val << " ";
            }
            captureFile << "\n";
        }

    }
    catch (exception &e)
    {
        cerr << "Exception writing captured data to file: " << e.what();
    }
}

void HandlingServer::loadTrajectories()
{
    // helper for loading
    DataFile dataFile;

    // load from file
    trajectory_vec trajectory;  // trajectory vector
    dataFile.loadSDFrames(trajectoryFile, trajectory);  // store file contents in trajectory vector

    if (trajectory.empty()) // we don't want to send empty trajectories
    {
        cerr << "Failure loading trajectory!" << endl;
        return;
    }

    // enqueue loaded trajectory (interpolated by robot)
    trajectoryQueue.push(trajectory);
    infoQueue.push(trajInfo);

    // sample some points and make new trajectory (interpolated by robot)
    trajectory_vec pointSampleTrajectory;
    pointSampleTrajectory.push_back(trajectory.front());
    pointSampleTrajectory.push_back(trajectory[10]);
    pointSampleTrajectory.push_back(trajectory[20]);
    pointSampleTrajectory.push_back(trajectory[30]);
    pointSampleTrajectory.push_back(trajectory.back());

    ++trajInfo[KUKA_TRAJID];
    trajectoryQueue.push(trajectory);
    infoQueue.push(trajInfo);

    // sample same points and enqueue as separate trajectories
    // these will be linear (not interpolated by robot)
    trajectory_vec pose;

    pose.push_back(trajectory.front());
    ++trajInfo[KUKA_TRAJID];
    trajectoryQueue.push(pose);
    infoQueue.push(trajInfo);

    pose[0] = trajectory[10];
    ++trajInfo[KUKA_TRAJID];
    trajectoryQueue.push(pose);
    infoQueue.push(trajInfo);

    pose[0] = trajectory[20];
    ++trajInfo[KUKA_TRAJID];
    trajectoryQueue.push(pose);
    infoQueue.push(trajInfo);

    pose[0] = trajectory[30];
    ++trajInfo[KUKA_TRAJID];
    trajectoryQueue.push(pose);
    infoQueue.push(trajInfo);

    pose[0] = trajectory.back();
    ++trajInfo[KUKA_TRAJID];
    trajectoryQueue.push(pose);
    infoQueue.push(trajInfo);

}
