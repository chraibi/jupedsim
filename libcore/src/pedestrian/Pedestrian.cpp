/**
 * \file        Pedestrian.cpp
 * \date        Sep 30, 2010
 * \version     v0.7
 * \copyright   <2009-2015> Forschungszentrum Jülich GmbH. All rights reserved.
 *
 * \section License
 * This file is part of JuPedSim.
 *
 * JuPedSim is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * JuPedSim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with JuPedSim. If not, see <http://www.gnu.org/licenses/>.
 *
 * \section Description
 *
 *
 **/
#include "Pedestrian.h"

#include "geometry/Building.h"
#include "geometry/SubRoom.h"
#include "geometry/WaitingArea.h"

#include <Logger.h>
#include <cassert>

// initialize the static variables
int Pedestrian::_agentsCreated         = 1;
double Pedestrian::_minPremovementTime = std::numeric_limits<double>::max();
AgentColorMode Pedestrian::_colorMode  = BY_VELOCITY;
std::vector<int> colors                = {
    0,
    255,
    35,
    127,
    90,
};


Pedestrian::Pedestrian()
{
    _id                              = _agentsCreated; //default id
    _exitIndex                       = -1;
    _group                           = -1;
    _desiredFinalDestination         = FINAL_DEST_OUT;
    _premovement                     = 0;
    _mass                            = 1;
    _tau                             = 0.5;
    _t                               = 1.0;
    _deltaT                          = 0.01;
    _ellipse                         = JEllipse();
    _v0                              = Point(0, 0);
    _v0UpStairs                      = 0.6;
    _v0DownStairs                    = 0.6;
    _v0EscalatorUpStairs             = 0.8;
    _v0EscalatorDownStairs           = 0.8;
    _smoothFactorUpStairs            = 15;
    _smoothFactorDownStairs          = 15;
    _smoothFactorEscalatorUpStairs   = 15;
    _smoothFactorEscalatorDownStairs = 15;
    _lastE0                          = Point(0, 0);
    _navLine                         = nullptr;
    _mentalMap                       = std::map<int, int>();
    _lastPosition                    = Point(J_NAN, J_NAN);
    // new orientation after 10 seconds, value is incremented
    _timeBeforeRerouting = 0.0;
    _newOrientationDelay = 0; //0 seconds, in steps
    _reroutingEnabled    = false;
    _router              = nullptr;
    _building            = nullptr;
    _agentsCreated++; //increase the number of object created
    _waitingPos = Point(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
}

Pedestrian::Pedestrian(const StartDistribution & agentsParameters, Building & building) :
    _group(agentsParameters.GetGroupId()),
    _desiredFinalDestination(agentsParameters.GetGoalId()),
    _premovement(agentsParameters.GetPremovementTime()),
    _lastPosition(),

    _router(building.GetRoutingEngine()->GetRouter(agentsParameters.GetRouterId())),
    _building(&building)
{
    _exitIndex           = -1;
    _id                  = _agentsCreated; //default id
    _mass                = 1;
    _tau                 = 0.5;
    _t                   = 1.0;
    _newOrientationDelay = 0; //0 seconds, in steps
    _ellipse             = JEllipse();
    _navLine             = nullptr;
    _router              = nullptr;
    _building            = nullptr;
    // new orientation after 10 seconds, value is incremented
    _timeBeforeRerouting     = 0.0;
    _reroutingEnabled        = false;
    _desiredFinalDestination = FINAL_DEST_OUT;
    _mentalMap               = std::map<int, int>();
    _deltaT                  = 0.01;
    _v0                      = Point(0, 0);
    _lastPosition            = Point(0, 0);
    _group                   = -1;
    _v0UpStairs              = 0.6;
    _v0DownStairs            = 0.6;
    _v0EscalatorUpStairs     = 0.8;
    _v0EscalatorDownStairs   = 0.8;
    _smoothFactorUpStairs    = agentsParameters.GetGroupParameters()->_smoothFactorUpStairs;
    _smoothFactorDownStairs  = agentsParameters.GetGroupParameters()->_smoothFactorDownStairs;
    _smoothFactorEscalatorUpStairs =
        agentsParameters.GetGroupParameters()->_smoothFactorEscalatorUpStairs;
    _smoothFactorEscalatorDownStairs =
        agentsParameters.GetGroupParameters()->_smoothFactorEscalatorDownStairs;
    _lastE0 = Point(0, 0);
    _agentsCreated++; //increase the number of object created
    _waitingPos = Point(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
}


Pedestrian::~Pedestrian()
{
    delete _navLine;
}

bool Pedestrian::InPremovement(double now)
{
    return _premovement >= now;
}

double Pedestrian::SelectV0(SubroomType type, double delta) const
{
    switch(type) {
        case SubroomType::ESCALATOR_UP:
            return _v0EscalatorUpStairs + _building->GetSubRoom(GetPos())->GetEscalatorSpeed();
        case SubroomType::ESCALATOR_DOWN:
            return _v0EscalatorDownStairs + _building->GetSubRoom(GetPos())->GetEscalatorSpeed();
        case SubroomType::STAIR:
            if(fabs(delta) < 1)
                return std::max(0., _ellipse.GetV0());
            else
                return (delta < 0) ? _v0DownStairs : _v0UpStairs;

        case SubroomType::FLOOR:
        case SubroomType::CORRIDOR:
        case SubroomType::ENTRANCE:
        case SubroomType::LOBBY:
        case SubroomType::DA:
        case SubroomType::UNKNOWN:
            return _ellipse.GetV0();
    }
}
double Pedestrian::SelectSmoothFactor(SubroomType type, double delta) const
{
    switch(type) {
        case SubroomType::ESCALATOR_UP:
            return _smoothFactorEscalatorUpStairs;
        case SubroomType::ESCALATOR_DOWN:
            return _smoothFactorEscalatorDownStairs;
        case SubroomType::STAIR:
            return (delta < 0) ? _smoothFactorDownStairs : _smoothFactorUpStairs;
        case SubroomType::FLOOR:
        case SubroomType::CORRIDOR:
        case SubroomType::ENTRANCE:
        case SubroomType::LOBBY:
        case SubroomType::DA:
        case SubroomType::UNKNOWN:
            return 15.0; /// magic number that works best so far!
    }
}

void Pedestrian::SetID(int i)
{
    _id = i;
    if(i <= 0) {
        throw std::logic_error("Invalid pedestrians ID: Pedestrian ID should be > 0.");
    }
}

void Pedestrian::SetTau(double tau)
{
    _tau = tau;
}

void Pedestrian::SetT(double T)
{
    _t = T;
}

void Pedestrian::SetEllipse(const JEllipse & e)
{
    _ellipse = e;
}

void Pedestrian::SetExitIndex(int i)
{
    _exitIndex                    = i;
    _mentalMap[GetUniqueRoomID()] = i;
}

void Pedestrian::SetExitLine(const NavLine * l)
{
    if(_navLine != nullptr)
        delete _navLine;
    if(l != nullptr) {
        _navLine = new NavLine(*l);
    }
}

void Pedestrian::SetPos(const Point & pos)
{
    _lastPosition = _ellipse.GetCenter();
    _ellipse.SetCenter(pos);
}

void Pedestrian::SetV(const Point & v)
{
    _ellipse.SetV(v);
}

void Pedestrian::SetSmoothFactorUpStairs(double c)
{
    _smoothFactorUpStairs = c;
}

void Pedestrian::SetSmoothFactorDownStairs(double c)
{
    _smoothFactorDownStairs = c;
}

void Pedestrian::SetSmoothFactorEscalatorUpStairs(double c)
{
    _smoothFactorEscalatorUpStairs = c;
}

void Pedestrian::SetSmoothFactorEscalatorDownStairs(double c)
{
    _smoothFactorEscalatorDownStairs = c;
}

void Pedestrian::SetV0Norm(
    double v0,
    double v0UpStairs,
    double v0DownStairs,
    double escalatorUp,
    double escalatorDown)
{
    _ellipse.SetV0(v0);
    _v0DownStairs          = v0DownStairs;
    _v0UpStairs            = v0UpStairs;
    _v0EscalatorUpStairs   = escalatorUp;
    _v0EscalatorDownStairs = escalatorDown;
}

void Pedestrian::SetDeltaT(double dt)
{
    _deltaT = dt;
}

int Pedestrian::GetID() const
{
    return _id;
}

double Pedestrian::GetMass() const
{
    return _mass;
}

double Pedestrian::GetTau() const
{
    return _tau;
}

double Pedestrian::GetT() const
{
    return _t;
}

const JEllipse & Pedestrian::GetEllipse() const
{
    return _ellipse;
}

int Pedestrian::GetExitIndex() const
{
    return _exitIndex;
}

NavLine * Pedestrian::GetExitLine() const
{
    return _navLine;
}

// return the unique subroom Identifier

int Pedestrian::GetUniqueRoomID() const
{
    auto [room_id, sub_room_id, _] = _building->GetRoomAndSubRoomIDs(GetPos());
    return room_id * 1000 + sub_room_id;
}

// returns the exit Id corresponding to the
// unique subroom identifier

int Pedestrian::GetNextDestination()
{
    if(_mentalMap.count(GetUniqueRoomID()) == 0) {
        return -1;
    }
    return _mentalMap[GetUniqueRoomID()];
}

Point Pedestrian::GetLastE0() const
{
    return _lastE0;
}
void Pedestrian::SetLastE0(Point E0)
{
    _lastE0 = E0;
}

const Point & Pedestrian::GetPos() const
{
    return _ellipse.GetCenter();
}

const Point & Pedestrian::GetV() const
{
    return _ellipse.GetV();
}

const Point & Pedestrian::GetV0() const
{
    return _v0;
}


double Pedestrian::GetV0Norm() const
{
    // @todo: we need to know the difference of the ped_elevation to the old_nav_elevation, and use this in the function f.
    //detect the walking direction based on the elevation
    SubRoom * sub        = _building->GetSubRoom(GetPos());
    double ped_elevation = sub->GetElevation(_ellipse.GetCenter());
    if(_navLine == nullptr) { // this might happen when agents are spawn by sources.
        return std::max(0., _ellipse.GetV0());
    }
    const Point & target = _navLine->GetCentre();
    double nav_elevation = sub->GetElevation(target);
    double delta         = nav_elevation - ped_elevation;
    auto subType         = sub->GetType();
    double smoothFactor  = SelectSmoothFactor(subType, delta);
    double v0            = SelectV0(subType, delta);
    double z1            = sub->GetMaxElevation();
    double z0            = sub->GetMinElevation();
    double alpha         = acos(sub->GetCosAngleWithHorizontal());
    double f =
        2.0 / (1 + exp(-smoothFactor * alpha * (z1 - ped_elevation) * (z1 - ped_elevation))) - 1;
    double g =
        2.0 / (1 + exp(-smoothFactor * alpha * (z0 - ped_elevation) * (z0 - ped_elevation))) - 1;

    double walking_speed = (1 - f * g) * _ellipse.GetV0() + f * g * v0;
    return walking_speed;
}

// get axis in the walking direction
double Pedestrian::GetLargerAxis() const
{
    return _ellipse.GetEA();
}
// get axis in the shoulder direction = orthogonal to the walking direction
double Pedestrian::GetSmallerAxis() const
{
    return _ellipse.GetEB();
}

void Pedestrian::SetPhiPed()
{
    double cosPhi;
    double sinPhi;
    double vx = GetV()._x;
    double vy = GetV()._y;

    if(fabs(vx) > J_EPS || fabs(vy) > J_EPS) {
        double normv = sqrt(vx * vx + vy * vy);
        cosPhi       = vx / normv;
        sinPhi       = vy / normv;
    } else {
        cosPhi = GetEllipse().GetCosPhi();
        sinPhi = GetEllipse().GetSinPhi();
    }
    _ellipse.SetCosPhi(cosPhi);
    _ellipse.SetSinPhi(sinPhi);
}


void Pedestrian::InitV0(const Point & target)
{
    const Point & pos = GetPos();
    Point delta       = target - pos;

    _v0 = delta.Normalized();
}


const Point & Pedestrian::GetV0(const Point & target)
{
    // Molification around the targets makes little sense
    const Point & pos = GetPos();
    Point delta       = target - pos;
    Point new_v0      = delta.Normalized();

    double t = _newOrientationDelay++ * _deltaT;

    //Handover new target
    _v0 = _v0 + (new_v0 - _v0) * (1 - exp(-t / _tau));
    return _v0;
}

void Pedestrian::SetSmoothTurning()
{
    _newOrientationDelay = 0;
}

void Pedestrian::UpdateReroutingTime()
{
    _timeBeforeRerouting -= _deltaT;
}

void Pedestrian::RerouteIn(double time)
{
    _reroutingEnabled    = true;
    _timeBeforeRerouting = time;
}

bool Pedestrian::IsReadyForRerouting() const
{
    return (_reroutingEnabled && (_timeBeforeRerouting <= 0.0));
}


int Pedestrian::GetGroup() const
{
    return _group;
}

void Pedestrian::SetGroup(int group)
{
    _group = group;
}

void Pedestrian::ResetRerouting()
{
    _reroutingEnabled    = false;
    _timeBeforeRerouting = -1.00;
}

double Pedestrian::GetDistanceToNextTarget() const
{
    return (_navLine->DistTo(GetPos()));
}

void Pedestrian::SetFinalDestination(int finale)
{
    _desiredFinalDestination = finale;
}

int Pedestrian::GetFinalDestination() const
{
    return _desiredFinalDestination;
}

std::string Pedestrian::GetPath()
{
    std::map<int, int>::iterator iter;
    std::string path;

    for(iter = _mentalMap.begin(); iter != _mentalMap.end(); iter++) {
        std::stringstream ss; //create a stringstream
        ss << iter->first / 1000 << ":" << iter->second
           << ">"; //@todo:ar.graf: has this to do with roomNr*1000+subroom and is now wrong?
        path.append(ss.str());
    }
    return path;
}

void Pedestrian::SetRouter(Router * router)
{
    _router = router;
}

int Pedestrian::GetRouterID() const
{
    return _router_id;
}


double Pedestrian::GetV0UpStairsNorm() const
{
    return _v0UpStairs;
}
double Pedestrian::GetV0DownStairsNorm() const
{
    return _v0DownStairs;
}
double Pedestrian::GetV0EscalatorUpNorm() const
{
    return _v0EscalatorUpStairs;
}
double Pedestrian::GetV0EscalatorDownNorm() const
{
    return _v0EscalatorDownStairs;
}


double Pedestrian::GetSmoothFactorUpStairs() const
{
    return _smoothFactorUpStairs;
}
double Pedestrian::GetSmoothFactorDownStairs() const
{
    return _smoothFactorDownStairs;
}
double Pedestrian::GetSmoothFactorUpEscalators() const
{
    return _smoothFactorEscalatorUpStairs;
}
double Pedestrian::GetSmoothFactorDownEscalators() const
{
    return _smoothFactorEscalatorDownStairs;
}

int Pedestrian::FindRoute()
{
    if(_router == nullptr) {
        LOG_ERROR("One or more routers does not exist! Check your router_ids");
        return -1;
    }
    return _router->FindExit(this);
}

double Pedestrian::GetElevation() const
{
    return _building->GetSubRoom(GetPos())->GetElevation(GetPos());
}

void Pedestrian::SetPremovementTime(double pretime)
{
    if(pretime < _minPremovementTime) {
        _minPremovementTime = pretime;
    }
    _premovement = pretime;
}

double Pedestrian::GetMinPremovementTime()
{
    return _minPremovementTime;
}

double Pedestrian::GetPremovementTime() const
{
    return _premovement;
}

const Building * Pedestrian::GetBuilding()
{
    return _building;
}

void Pedestrian::SetBuilding(Building * building)
{
    _building = building;
}

void Pedestrian::SetColorMode(AgentColorMode mode)
{
    _colorMode = mode;
}

int Pedestrian::GetAgentsCreated()
{
    return _agentsCreated;
}

int Pedestrian::GetColor() const
{
    //default color is by velocity
    std::string key;

    switch(_colorMode) {
        case BY_VELOCITY: {
            int color = -1;
            double v0 = _ellipse.GetV0();
            if(v0 != 0.0) {
                double v = GetV().Norm();
                color    = static_cast<int>(v / v0 * 255);
            }
            return color;
        }

        case BY_ROUTER:
        case BY_ROUTE: {
            key = std::to_string(_router_id);
        } break;

        case BY_GROUP: {
            key = std::to_string(_group); // @todo find a better solution to get
                                          // colors clearly distinguishable form
                                          // each other
            return (colors[_group % colors.size()]);
        }

        case BY_FINAL_GOAL: {
            key = std::to_string(_desiredFinalDestination);
        } break;

        case BY_INTERMEDIATE_GOAL: {
            key = std::to_string(_exitIndex);
        } break;

        default:
            break;
    }

    std::hash<std::string> hash_fn;
    return hash_fn(key) % 255;
}


int Pedestrian::GetLastGoalID() const
{
    return _lastGoalID;
}

bool Pedestrian::IsInsideWaitingAreaWaiting(double time) const
{
    if(_insideGoal) {
        auto itr = _building->GetAllGoals().find(_desiredFinalDestination);
        if(itr != _building->GetAllGoals().end()) {
            Goal * goal = itr->second;
            if(auto wa = dynamic_cast<WaitingArea *>(goal)) {
                return wa->IsWaiting(time, _building);
            }
        }
    }
    return false;
}

void Pedestrian::EnterGoal()
{
    _insideGoal = true;
    _lastGoalID = _desiredFinalDestination;
}

void Pedestrian::LeaveGoal()
{
    _insideGoal = false;
}

bool Pedestrian::IsWaiting() const
{
    return _waiting;
}

void Pedestrian::StartWaiting()
{
    _waiting = true;
}

void Pedestrian::EndWaiting()
{
    _waiting       = false;
    _waitingPos._x = std::numeric_limits<double>::max();
    _waitingPos._y = std::numeric_limits<double>::max();
}

const Point & Pedestrian::GetWaitingPos() const
{
    return _waitingPos;
}

void Pedestrian::SetWaitingPos(const Point & waitingPos)
{
    _waitingPos = waitingPos;
}

Point Pedestrian::GetLastPosition() const
{
    return _lastPosition;
}

std::string Pedestrian::ToString() const
{
    std::string message = fmt::format(
        FMT_STRING("------> ped {:d} <-------\n"
                   ">> Destination [ {:d} ]\n"
                   ">> Final Destination [ {:d} ]\n"
                   ">> Position [{:.2f}, {:.2f}]\n"
                   ">> Velocity [{:.2f}, {:.2f}]  Norm = [{:.2f}]\n"),
        _id,
        _exitIndex,
        _desiredFinalDestination,
        GetPos()._x,
        GetPos()._y,
        GetV()._x,
        GetV()._y,
        GetV().Norm());

    return message;
}

std::ostream & operator<<(std::ostream & out, const Pedestrian & pedestrian)
{
    return out << pedestrian.ToString();
}
