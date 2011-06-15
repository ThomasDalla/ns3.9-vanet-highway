/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005-2009 Old Dominion University [ARBABI]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Hadi Arbabi <marbabi@cs.odu.edu>
 */

#include <iostream>
#include <sstream>
#include "Controller.h"
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <fstream>

using namespace std;
using namespace ns3;

 double string_to_double( const std::string& s )
 {
   std::istringstream i(s);
   double x;
   if (!(i >> x))
     return 0;
   return x;
 }

namespace ns3
{
  Controller::Controller()
  {
	  T=-1.0;
	  Plot=false;
	  RecordAmbuPos=false;
	  AmbuFile="./ambuX";
	  AmbuFileContent="";
	  this->destinationReached=false;
	  this->timeToReachDest=0.0;
	  this->ambulanceId = 0;
	  this->srcX  = 0.0;
	  this->destX = 0.0;
	  this->startTime = 0.0;
	  this->emergencyDbThreshold = -0.5;
	  this->dstForDriverToReact = 100;
	  this->laneChangeNb=0;
	  this->laneChangeITS=0;
  }
  Controller::Controller(Ptr<Highway> highway)
  {
    this->highway=highway;
  }

  void Controller::SetHighway(Ptr<Highway> highway)
  {
    this->highway=highway;
  }

  Ptr<Highway> Controller::GetHighway()
  {
    return this->highway;
  }

  bool Controller::InitVehicle(Ptr<Highway> highway, int& VID)
  {
      // Block the road with warning, 500 meters away, at the right most lane [lane=0, dir=1]
      /*
      Ptr<Obstacle> obstacle=CreateObject<Obstacle>();
      obstacle->SetupWifi(highway->GetWifiHelper(), highway->GetYansWifiPhyHelper(), highway->GetNqosWifiMacHelper());
      obstacle->SetVehicleId(VID++);
      obstacle->SetDirection(1);
      obstacle->SetLane(0);
      obstacle->SetPosition(Vector(1000, highway->GetYForLane(0,1), 0));
      obstacle->SetLength(8);
      obstacle->SetWidth(2);
      obstacle->SetReceiveCallback(highway->GetReceiveDataCallback());
      highway->AddVehicle(obstacle);
      Simulator::Schedule(Seconds(0.0), &Controller::BroadcastWarning, this, obstacle);
      */
/*
      // Create an ambulance car moving toward the blocked spot, remember its VehicleId will be 2
      // We also show how we can assign this car new wifiphy instead of using default highway wifi,
      // for example a ambulance with a stronger transmission.
      // Remember all the vehicles must use the same shared wifi channel which already hold by highway.
      // Highway TxPower default value is 21.0 for (250m-300m) range, 30.0 makes the ambulance has higher range.
      // Also let ambulance car has a higher max speed.
      YansWifiPhyHelper ambulancePhyHelper = YansWifiPhyHelper::Default();
      ambulancePhyHelper.SetChannel(highway->GetWifiChannel());
      ambulancePhyHelper.Set("TxPowerStart",DoubleValue(30.0));
      ambulancePhyHelper.Set("TxPowerEnd",DoubleValue(30.0));


      Ptr<Vehicle> ambulance=CreateObject<Vehicle>();
      ambulance->SetupWifi(highway->GetWifiHelper(), ambulancePhyHelper, highway->GetNqosWifiMacHelper());
      ambulance->SetVehicleId(VID++);
      ambulance->SetDirection(1);
      ambulance->SetLane(1);
      ambulance->SetPosition(Vector(0.0, highway->GetYForLane(1,1), 0));
      ambulance->SetVelocity(0.0);
      ambulance->SetAcceleration(0.0);
      Ptr<Model> ambulanceModel=highway->CreateSedanModel();
      ambulanceModel->SetDesiredVelocity(40.0);  // max speed 36(m/s)
      ambulance->SetModel(ambulanceModel);          // or common sedan model: highway->GetSedanModel()
      ambulance->SetLaneChange(highway->GetSedanLaneChange());
      ambulance->SetLength(4);
      ambulance->SetWidth(2);
      ambulance->SetReceiveCallback(highway->GetReceiveDataCallback());
      highway->AddVehicle(ambulance);
      Simulator::Schedule(Seconds(0.0), &Controller::BroadcastWarning, this, ambulance);
*/
/*
      // create a vehicle on the road with wifi to setup the wifi on the highway
      YansWifiPhyHelper wifiVehiclePhyHelper = YansWifiPhyHelper::Default();
      wifiVehiclePhyHelper.SetChannel(highway->GetWifiChannel());
      wifiVehiclePhyHelper.Set("TxPowerStart",DoubleValue(30.0));
      wifiVehiclePhyHelper.Set("TxPowerEnd",DoubleValue(30.0));

      Ptr<Vehicle> wifiVehicle=CreateObject<Vehicle>();
      wifiVehicle->SetupWifi(highway->GetWifiHelper(), wifiVehiclePhyHelper, highway->GetNqosWifiMacHelper());
      wifiVehicle->SetVehicleId(VID++);
      wifiVehicle->SetDirection(1);
      wifiVehicle->SetLane(1);
      wifiVehicle->SetPosition(Vector(2000.0, highway->GetYForLane(1,1), 0));
      wifiVehicle->SetVelocity(0.0);
      wifiVehicle->SetAcceleration(0.0);
      Ptr<Model> wifiVehicleModel=highway->CreateSedanModel();
      wifiVehicleModel->SetDesiredVelocity(40.0);  // max speed 36(m/s)
      wifiVehicle->SetModel(wifiVehicleModel);          // or common sedan model: highway->GetSedanModel()
      wifiVehicle->SetLaneChange(highway->GetSedanLaneChange());
      wifiVehicle->SetLength(4);
      wifiVehicle->SetWidth(2);
      wifiVehicle->SetReceiveCallback(highway->GetReceiveDataCallback());
      highway->AddVehicle(wifiVehicle);
*/

      Simulator::Schedule(Seconds(1000.0), &Controller::StartAmbulanceVehicle, this, highway);
      highway->SetAutoInject(true);
      this->JsonOutput("ambuFile", AmbuFile);
      //cout << "\"ambuFileChar\":\"" << AmbuFile.c_str() << "\"," << endl;

      // Return true: a signal to highway that the lane lists (queues) in where obstacles and vehicles are being added
      // must be sorted based on their positions.
      // Return false: to ignore sorting (do not return false when vehicles are manually added to the highway).
      return true;
  }

  void Controller::JsonOutput(string name, int value)
  {
	  cout << "\"" << name << "\":" << value << "," << endl;
  }

  void Controller::JsonOutput(string name, double value)
  {
	  cout << "\"" << name << "\":" << value << "," << endl;
  }

  void Controller::JsonOutput(string name, string value)
    {
  	  cout << "\"" << name << "\":\"" << value << "\"," << endl;
    }

  void Controller::JsonOutputInitVehicles(int lane, std::list<Ptr<Vehicle> > vehicles)
  {
	    // vehicles on lane 0
		string vehiclesPos, vehiclesSpeed;
		ostringstream laneStr;
		laneStr << lane;
		string posName = "vehiclesPosLane";
		posName.append(laneStr.str());
		string speedName = "vehiclesSpeedLane";
		speedName.append(laneStr.str());
		string averageGapName = "averageGapOnLane";
		averageGapName.append(laneStr.str());
		averageGapName.append("_New");
		string minGapName = "minGapLane";
		minGapName.append(laneStr.str());
		string maxGapName = "maxGapLane";
		maxGapName.append(laneStr.str());

		bool nouveau=true;
		int k=0;
		double previousPos=0;
		double gapTotal = 0;
		double minGap = 10000;
		double maxGap = 0;
		double currentGap = 0;
		double previousGap = 0;
		while (vehicles.size()>0)
		{
		  ostringstream pos, vel;
		  Ptr<Vehicle> v = vehicles.back();
		  vehicles.pop_back();
		  double currentPos = v->GetPosition().x;
		  pos << currentPos;
		  vel << v->GetVelocity();
		  if (nouveau==false) {
			  k++;
			  currentGap = currentPos-previousPos;
			  gapTotal += currentGap;
			  if (currentGap>maxGap)
				  maxGap = currentGap;
			  if (currentGap<minGap)
				  minGap = currentGap;
			  vehiclesPos.append(" ");
			  vehiclesSpeed.append(" ");
		  } else {
			  nouveau= false;
		  }
		  vehiclesPos.append(pos.str());
		  vehiclesSpeed.append(vel.str());
		  previousPos = currentPos;
		  previousGap = currentGap;
		}
		double averageGap= 1.0*gapTotal/k;
		this->JsonOutput(posName, vehiclesPos);
		this->JsonOutput(speedName, vehiclesSpeed);
		this->JsonOutput(averageGapName, averageGap);
		this->JsonOutput(minGapName, minGap);
		this->JsonOutput(maxGapName, maxGap);
  }

  void Controller::StartAmbulanceVehicle(Ptr<Highway> highway)
  {
      double xMin = 500;
      double xMax = 2000;
      int lane= 0;
      int dir= 1;
      list< Ptr< Vehicle > > vehicles = highway->FindVehiclesInSegment(xMin,xMax, lane, dir);
      int vehiclesFoundNb = vehicles.size();
      if (vehiclesFoundNb>0)
      {
          int vehicle_i = int ( floor(vehiclesFoundNb/2.0) );

          int i=0;
          while (i<vehicle_i)
          {
              vehicles.pop_front();
              i++;
          }

          Ptr<Vehicle> ambulance=vehicles.front();
          this->ambulanceId = ambulance->GetVehicleId();
          double ambuX = ambulance->GetPosition().x;
          if (!Plot)
          {
//              cout << "I chose vehicle " << vehicle_i << " among " << vehiclesFoundNb << " vehicles."<< endl;
//              cout << "ID " << this->ambulanceId << " Position: " << ambuX << endl;
              this->JsonOutput("vehicleChosen", vehicle_i);
              this->JsonOutput("vehicleChosenAmong", vehiclesFoundNb);
              this->JsonOutput("ambuId", this->ambulanceId);
              this->JsonOutput("ambuStartX", ambuX);
              // Calculate the density
              //double length=highway->GetHighwayLength()-ambuX-1.0;
              list< Ptr< Vehicle > > vehicles0 = highway->FindVehiclesInSegment(ambuX+1.0,highway->GetHighwayLength(), 0, dir);
              list< Ptr< Vehicle > > vehicles1 = highway->FindVehiclesInSegment(ambuX+1.0,highway->GetHighwayLength(), 1, dir);
              Ptr<Vehicle> last0 = vehicles0.front();
              Ptr<Vehicle> last1 = vehicles1.front();
              double density0 = (last0->GetPosition().x-ambuX-1.0)/vehicles0.size();
              double density1 = (last1->GetPosition().x-ambuX-1.0)/vehicles1.size();
              this->JsonOutput("vehiclesOnLane0", (int) vehicles0.size());
              this->JsonOutput("averageGapOnLane0", density0);
              this->JsonOutput("lastVehicleXOnLane0", last0->GetPosition().x);
              this->JsonOutput("vehiclesOnLane1", (int) vehicles1.size());
              this->JsonOutput("averageGapOnLane1", density1);
              this->JsonOutput("lastVehicleXOnLane1", last1->GetPosition().x);
              this->JsonOutputInitVehicles(0, vehicles0);
              this->JsonOutputInitVehicles(1, vehicles1);
          }
          YansWifiPhyHelper ambulancePhyHelper = YansWifiPhyHelper::Default();
          ambulancePhyHelper.SetChannel(highway->GetWifiChannel());
          ambulancePhyHelper.Set("TxPowerStart",DoubleValue(50.0));
          ambulancePhyHelper.Set("TxPowerEnd",DoubleValue(50.0));
          ambulance->IsEquipped = true;
          ambulance->SetupWifi(highway->GetWifiHelper(), ambulancePhyHelper, highway->GetNqosWifiMacHelper());
          //ambulance->SetVehicleId();
          //ambulance->SetDirection(1);
          //ambulance->SetLane(1);
          //ambulance->SetPosition(Vector(0.0, highway->GetYForLane(1,1), 0));
          //ambulance->SetVelocity(40.0);
          ambulance->SetAcceleration(5.0);
          Ptr<Model> ambulanceModel=highway->CreateSedanModel();
          ambulanceModel->SetDesiredVelocity(47.0);  // max speed 36(m/s)
          ambulanceModel->SetDeltaV(10.0);
          ambulanceModel->SetAcceleration(3.0);
          ambulanceModel->SetDeceleration(3.0);
          ambulance->SetModel(ambulanceModel);          // or common sedan model: highway->GetSedanModel()
          Ptr<LaneChange> ambulanceLaneChangeModel = highway->CreateSedanLaneChangeModel();
          ambulanceLaneChangeModel->SetDbThreshold(100.0); // no lane change
          ambulance->SetLaneChange(ambulanceLaneChangeModel);
          ambulance->SetLength(4);
          ambulance->SetWidth(2);
          ambulance->SetReceiveCallback(highway->GetReceiveDataCallback());
          ambulance->SetDevTxTraceCallback(highway->GetDevTxTraceCallback());
          ambulance->SetDevRxTraceCallback(highway->GetDevRxTraceCallback());
          ambulance->SetPhyRxOkTraceCallback(highway->GetPhyRxOkTraceCallback());
          ambulance->SetPhyRxErrorTraceCallback(highway->GetPhyRxErrorTraceCallback());
          ambulance->SetPhyTxTraceCallback(highway->GetPhyTxTraceCallback());
          ambulance->SetPhyStateTraceCallback(highway->GetPhyStateTraceCallback());
          Simulator::Schedule(Seconds(1), &Controller::BroadcastWarning, this, ambulance);

          // start recording data
          double now=Simulator::Now().GetSeconds();
          this->startTime = now;
          this->srcX  = ambuX;
          this->destX = ambuX + 10000;
          if (!Plot)
          {
//              cout << "at t=" << now << " AMBULANCE (" << this->ambulanceId <<
//              ") START RECORDING TIME! Position: " << ambuX << "m"
//              << " Speed: " << ambulance->GetVelocity() << endl;
              this->JsonOutput("startRecordingTime", now);
              this->JsonOutput("ambuXwhenStart",ambuX);
              this->JsonOutput("ambuSpeedWhenStart",ambulance->GetVelocity());
          }

      }

      else
      {
          if (!Plot) {
        	  this->JsonOutput("vehicleChosenAmong", vehiclesFoundNb);
        	  this->JsonOutput("vehicleTotal", highway->GetLastVehicleId());
            //cout << "NO VEHICLE FOUND IN RANGE [" << xMin << "," << xMax << "]" << endl;
          }
          Simulator::Stop();
      }

  }

  void Controller::AddAmbulanceVehicle(Ptr<Highway> highway)
  {

      //       select a random vehicle around position x=1000
      //       and define it as an ambulance in service

      //this->ambulanceId = highway->GetLastVehicleId()+1;
      this->ambulanceId = 0;

      YansWifiPhyHelper ambulancePhyHelper = YansWifiPhyHelper::Default();
      ambulancePhyHelper.SetChannel(highway->GetWifiChannel());
      ambulancePhyHelper.Set("TxPowerStart",DoubleValue(35.0));
      ambulancePhyHelper.Set("TxPowerEnd",DoubleValue(35.0));

      Ptr<Vehicle> ambulance=CreateObject<Vehicle>();
      ambulance->SetupWifi(highway->GetWifiHelper(), ambulancePhyHelper, highway->GetNqosWifiMacHelper());
      ambulance->SetVehicleId(this->ambulanceId);
      ambulance->SetDirection(1);
      ambulance->SetLane(1);
      ambulance->SetPosition(Vector(0.0, highway->GetYForLane(1,1), 0));
      ambulance->SetVelocity(40.0);
      ambulance->SetAcceleration(2.0);
      Ptr<Model> ambulanceModel=highway->CreateSedanModel();
      ambulanceModel->SetDesiredVelocity(47.0);  // max speed 36(m/s)
      ambulance->SetModel(ambulanceModel);          // or common sedan model: highway->GetSedanModel()
      Ptr<LaneChange> ambulanceLaneChangeModel = highway->CreateSedanLaneChangeModel();
      ambulanceLaneChangeModel->SetDbThreshold(100.0);
      ambulance->SetLaneChange(ambulanceLaneChangeModel);
      ambulance->SetLength(4);
      ambulance->SetWidth(2);
      ambulance->SetReceiveCallback(highway->GetReceiveDataCallback());
      highway->AddVehicle(ambulance);
//      Simulator::Schedule(Seconds(1), &Controller::BroadcastWarning, this, ambulance);

  }

  bool Controller::ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt)
  {
	if (RecordAmbuPos==true && vehicle->GetVehicleId()==this->ambulanceId)
	{
		/*
		ofstream outputFile;
		outputFile.open(AmbuFile.c_str(), ios_base::app);
		outputFile << now << " " << vehicle->GetPosition().x << endl;
		*/
		ostringstream pos, now;
		pos << vehicle->GetPosition().x;
		now << Simulator::Now().GetSeconds();
		AmbuFileContent.append(now.str());
		AmbuFileContent.append(" ");
		AmbuFileContent.append(pos.str());
		AmbuFileContent.append("\n");
	}
    // we aim to create outputs which are readable by gnuplot for visulization purpose
    // this can be happen at beginning of each simulation step here.
	else if(Plot==true)
    {
      bool newStep=false;
      double now=Simulator::Now().GetHighPrecision().GetDouble();
      if(now > T)
      {
        T = now;
        newStep=true;
      }

      if(newStep==true)
      {
        if(T!=0.0)
        {
          cout << "e" << endl;
          //cout << "pause " << dt << endl;
        }
        float xrange = highway->GetHighwayLength();
        float yrange = highway->GetLaneWidth()*highway->GetNumberOfLanes();
        if(highway->GetTwoDirectional()) yrange=2*yrange + highway->GetMedianGap();
        cout << "set xrange [0:"<< xrange <<"]" << endl;
        cout << "set yrange [0:"<< yrange <<"]" << endl;
        cout << "plot '-' w points" << endl;
        newStep=false;
      }
      if(newStep==false)
      {
        cout << vehicle->GetPosition().x << " " << vehicle->GetPosition().y << endl;
      }
    }
/*
    if (!this->destinationReached && vehicle->GetVehicleId()==this->ambulanceId && this->startTime<=0 && vehicle->GetPosition().x >=this->srcX)
    {
        //this->startTime = Simulator::Now().GetHighPrecision().GetDouble();
        double now=Simulator::Now().GetSeconds();
        this->startTime = now;
        if (!Plot)
            cout << "at t=" << now << " AMBULANCE (" << vehicle->GetVehicleId() <<
            ") START RECORDING TIME! Position: " << vehicle->GetPosition().x << "m"
            << " Speed: " << vehicle->GetVelocity() << endl;
        Simulator::Schedule(Seconds(1), &Controller::BroadcastWarning, this, vehicle);
    }
*/

    // Change lane for a driver without wifi
    if(vehicle->GetVehicleId()!=this->ambulanceId && this->startTime>0 && vehicle->GetLaneChange()->GetDbThreshold()!=this->emergencyDbThreshold && !this->destinationReached)
    {
        Ptr<Vehicle> ambulance = highway->FindVehicle(this->ambulanceId);
        double ambuX    = ambulance->GetPosition().x;
        double myX      = vehicle->GetPosition().x;
        double ambuLane = ambulance->GetLane();
        double myLane   = vehicle->GetLane();
        double diff = myX - ambuX;
        if (myLane==ambuLane && myX >= ambuX && diff<=vehicle->GetDetectsVehicleDistance())
        {
            if (false&&!Plot)
            {
            	double now=Simulator::Now().GetSeconds();
                cout << "at t=" << now << " veh " << vehicle->GetVehicleId() << " at " << diff << "m of the ambu tries to change lane (speed=" << vehicle->GetVelocity() << ")" << endl;
            }
            this->AskChangeLane(highway, vehicle);
        }
    }

    // to record when the ambulance reaches its destination
    else if(vehicle->GetVehicleId()==this->ambulanceId && vehicle->GetPosition().x >=this->destX && this->startTime>0 && !this->destinationReached)
    {
        this->destinationReached = true;
        double now=Simulator::Now().GetSeconds();
        //this->timeToReachDest = Simulator::Now().GetHighPrecision().GetDouble()-this->startTime;
        this->timeToReachDest = now-this->startTime;
        if (!Plot)
        {
//            cout << "at t=" << now << " AMBULANCE REACHED DEST in " << this->timeToReachDest
//            << "s! Position: " << vehicle->GetPosition().x << "m" << " Speed: " << vehicle->GetVelocity() << endl;
            this->JsonOutput("ambuReachedDestTime", now);
            this->JsonOutput("timeToReachDest", this->timeToReachDest);
            this->JsonOutput("ambuXwhenReached", vehicle->GetPosition().x);
            this->JsonOutput("ambuSpeedWhenReached", vehicle->GetVelocity());
            this->JsonOutput("laneChangeNb", this->laneChangeNb);
            this->JsonOutput("laneChangeITS", this->laneChangeITS);
        }
        if (RecordAmbuPos==true && vehicle->GetVehicleId()==this->ambulanceId)
        {
//			ofstream outputFile;
//			outputFile.open(AmbuFile.c_str(), ios_base::app);
        	ofstream outputFile(AmbuFile.c_str());
			outputFile << AmbuFileContent;
        }
        Simulator::Stop();
//        Simulator::Destroy();
      //vehicle->SetAcceleration(-2.0);
      // return true: a signal to highway that we aim to manually control the vechile
      //return true;
    }

    // return false: a signal to highway that lets the vehicle automatically be handled (using IDM/MOBIL rules)
    return false;
  }

  void Controller::BroadcastWarning(Ptr<Vehicle> veh)
  {
    double distance_to_dest = this->destX - veh->GetPosition().x;
    if (distance_to_dest>0) {
      stringstream msg;
      msg << veh->GetPosition().x << " " << veh->GetDirection() << " " << veh->GetLane();
/*
      msg << "ambu"//veh->GetVehicleId()
          //<< " " << veh->GetPosition().x
          << " is on the highway at x=" << veh->GetPosition().x
          << " dir=" << veh->GetDirection()
          << " lane=" << veh->GetLane();
*/
      if (false&&!Plot)
      {
        int vid=veh->GetVehicleId();
        double now=Simulator::Now().GetSeconds();
        cout << "at t=" << now << " vehicle " << vid << " is at pos=" << veh->GetPosition().x << ", dst to dest=" << distance_to_dest
        << "m to the destination, on lane " << veh->GetLane() << " at speed " << veh->GetVelocity() << "m/s"
        << " (desired:" << veh->GetModel()->GetDesiredVelocity() << ")" << endl;
      }
      Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), msg.str().length());
      //cout << "Broadcast warning get Broadcast address..." << endl;
      veh->SendTo(veh->GetBroadcastAddress(), packet);
      //cout << " done! (" << veh->GetBroadcastAddress() << ")" << endl;

      Simulator::Schedule(Seconds(5.0),&Controller::BroadcastWarning, this, veh);
    }
  }

  void Controller::ReceiveData(Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address)
  {
      int vid=veh->GetVehicleId();
      if (vid == this->ambulanceId)
      {
          if (false&&!Plot)
          {
              double now=Simulator::Now().GetSeconds();
              cout << "t=" << now << " veh " << vid << " received its own message (ambuId = "<< this->ambulanceId << ") :s" << endl;
          }
      }
      else
      {
        string data=string((char*)packet->PeekData());
        stringstream ss (stringstream::in | stringstream::out);

        double obs_id, obs_x;
        ss << data;
        ss >> obs_id;
        ss >> obs_x;

        int lane = veh->GetLane();
        double vehX = veh->GetPosition().x;
        vector<string> strs;
        boost::split(strs, data, boost::is_any_of("\t "));
        double ambuX = string_to_double(strs.at(0));
        double ambuLane = string_to_double(strs.at(2));
        double now=Simulator::Now().GetSeconds();
        //cout << "ambuX=" << ambuX << "ambuLane=" << ambuLane << endl;
        if (lane==ambuLane && ambuX < vehX) {
            if(false&&!Plot)
            {
                cout << "t=" << now << " veh " << vid << " rec msg=[" << data << "] own pos="
                << vehX << "m lane=" << lane << " speed= " << veh->GetVelocity() << "m/s"
                << " (desired:" << veh->GetModel()->GetDesiredVelocity() << ")"<<endl;
                cout << "\tOh my! I should change lane!!! \tdbThreshold=" << veh->GetLaneChange()->GetDbThreshold()
                << " politness=" << veh->GetLaneChange()->GetPolitenessFactor()
                << " gapMin=" << veh->GetLaneChange()->GetGapMin()
                << " biasRight= " << veh->GetLaneChange()->GetBiasRight() << endl;
            }
            this->laneChangeITS++;
            this->AskChangeLane(highway, veh);

        }

    }
  }

  void Controller::AskChangeLane(Ptr<Highway> highway, Ptr<Vehicle> veh)
  {
      // create a new model and assign it
	  this->laneChangeNb++;
      Ptr<LaneChange> newLaneChangeModel = highway->CreateSedanLaneChangeModel();
      newLaneChangeModel->SetDbThreshold(this->emergencyDbThreshold);
      newLaneChangeModel->SetGapMin(1);
      //newLaneChangeModel->SetPolitenessFactor(1);
      //newLaneChangeModel->SetBiasRight(100);
      veh->SetLaneChange(newLaneChangeModel);
      //std::list<Ptr<Vehicle> > vehi_list;
      //vehi_list.insert(vehi_list.begin(),veh);
      //this->highway->ChangeLane(&vehi_list);"
  }

}
