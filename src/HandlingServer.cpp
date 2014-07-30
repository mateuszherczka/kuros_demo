#include "HandlingServer.hpp"

HandlingServer::HandlingServer() {}

HandlingServer::~HandlingServer() {}

void HandlingServer::handleResponse()
{
    ++handledCount;

    // let's capture response stream to file for each trajectory

    robotStatus = response.info[KUKA_RSP_STATUS];

    switch (robotStatus)
    {
    case 0: // not connected or exiting
        cout << "Robot offline.\n";
        break;

    case 1: // BCO completed, ready
        printResponse();

        loadTrajectories(); // load trajectories

        sendNextTrajectory();

        break;

    case 2: // Received trajectory, about to move
        printResponse();

        if (!nowCapturing)
        {
            startCapturing();
        }
        break;

    case 4: // Finished trajectory, ready for next
        printResponse();

        if (nowCapturing)
        {
            finishCapturing();
        }

        // send next available trajectory
        sendNextTrajectory();

        break;

    case 5: //  means "streamed message"

        // capture frames from robot to RAM
        capturedFrames.push_back(response.frame);

        break;

    }

}

void HandlingServer::handleDisconnect()
{
    // we can call startListening() again if we want
    cout << "Server disconnected." << endl;
}

void HandlingServer::printResponse()
{
    cout << "------------------------------------------------" << endl;
    cout << "Response #" << handledCount << endl;

    response.printValues();

    cout << "------------------------------------------------" << endl;

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
    cout << "Saving trajectory id " << response.info[KUKA_RSP_TRAJID] << endl;
    std::string captureName = (boost::format("captured_%1%.txt") %response.info[KUKA_RSP_TRAJID]).str();
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
    trajectoryQueue.push(pointSampleTrajectory);
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
