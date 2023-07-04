using MissionPlanner.ArduPilot;
using MissionPlanner.Utilities;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using System;
using System.Collections.Generic;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;
using Vector3 = MissionPlanner.Utilities.Vector3;
using netDxf.Entities;

namespace MissionPlanner.Swarm
{
    /// <summary>
    /// Follow the leader
    /// </summary>
    class Formation : Swarm
    {
        Dictionary<MAVState, Vector3> offsets = new Dictionary<MAVState, Vector3>();

        private Dictionary<MAVState, Tuple<PID, PID, PID, PID>> pids =
            new Dictionary<MAVState, Tuple<PID, PID, PID, PID>>();

        private PointLatLngAlt masterpos = new PointLatLngAlt();
        private PointLatLngAlt oldmasterpos = null;
        private int masterpos_last_time_s = 0;

        public void setOffsets(MAVState mav, double x, double y, double z)
        {
            offsets[mav] = new Vector3(x, y, z);
            log.Info(mav.ToString() + " " + offsets[mav].ToString());
        }

        public Vector3 getOffsets(MAVState mav)
        {
            if (offsets.ContainsKey(mav))
            {
                return offsets[mav];
            }

            return new Vector3(offsets.Count, 0, 0);
        }

        public override void Update()
        {
            if (MainV2.comPort.MAV.cs.lat == 0 || MainV2.comPort.MAV.cs.lng == 0)
                return;

            if (Leader == null)
                Leader = MainV2.comPort.MAV;

            masterpos = new PointLatLngAlt(Leader.cs.lat, Leader.cs.lng, Leader.cs.alt, "");
        }

        double wrap_180(double input)
        {
            if (input > 180)
                return input - 360;
            if (input < -180)
                return input + 360;
            return input;
        }

        //convert Wgs84ConversionInfo to utm
        CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();

        IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;

        public override void SendCommand()
        {
            if (masterpos.Lat == 0 || masterpos.Lng == 0)
                return;

            //Console.WriteLine(DateTime.Now);
            //Console.WriteLine("Leader {0} {1} {2}", masterpos.Lat, masterpos.Lng, masterpos.Alt);

            int a = 0;
            foreach (var port in MainV2.Comports.ToArray())
            {
                foreach (var mav in port.MAVlist)
                {
                    if (mav == Leader)
                        continue;

                    PointLatLngAlt target = new PointLatLngAlt(masterpos);
                    if (oldmasterpos == null)
                    {
                        oldmasterpos = new PointLatLngAlt(masterpos);
                        return;
                    }

                    bool hasMaster = false;
                    if (oldmasterpos.GetDistance(masterpos) > 4.0f)
                    {
                        masterpos_last_time_s = DateTime.Now.toUnixTime();
                        oldmasterpos = new PointLatLngAlt(masterpos);
                        hasMaster = true;
                    }

                    try
                    {
                        int utmzone = (int)((masterpos.Lng - -186.0) / 6.0);

                        IProjectedCoordinateSystem utm = ProjectedCoordinateSystem.WGS84_UTM(utmzone,
                            masterpos.Lat < 0 ? false : true);

                        ICoordinateTransformation trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);

                        double[] pll1 = { target.Lng, target.Lat };

                        double[] p1 = trans.MathTransform.Transform(pll1);

                        double heading = -Leader.cs.yaw;

                        double length = offsets[mav].length();

                        var x = ((Vector3)offsets[mav]).x;
                        var y = ((Vector3)offsets[mav]).y;

                        // add offsets to utm
                        p1[0] += x * Math.Cos(heading * MathHelper.deg2rad) - y * Math.Sin(heading * MathHelper.deg2rad);
                        p1[1] += x * Math.Sin(heading * MathHelper.deg2rad) + y * Math.Cos(heading * MathHelper.deg2rad);

                        // convert back to wgs84
                        IMathTransform inversedTransform = trans.MathTransform.Inverse();
                        double[] point = inversedTransform.Transform(p1);

                        target.Lat = point[1];
                        target.Lng = point[0];
                        target.Alt += ((Vector3)offsets[mav]).z;

                        // ArduPlane Type
                        if (mav.cs.firmware == Firmwares.ArduPlane)
                        {
                            double targetyaw = 0.0f;
                            double targetspeed = 0.0f;
                            double dist = 0.0f;
                            int err_time = DateTime.Now.toUnixTime() - masterpos_last_time_s;
                            if (hasMaster || err_time < 2)
                            {
                                // get distance from target position
                                dist = target.GetDistance(mav.cs.Location);

                                // get bearing to target
                                var targyaw = mav.cs.Location.GetBearing(target);
                                if (err_time >= 1 && Math.Abs(targyaw-Leader.cs.yaw)>=5.0f)
                                {
                                    targyaw = Leader.cs.yaw;
                                }

                                Tuple<PID> pid;

                                if (pids.ContainsKey(mav))
                                {
                                    pid = pids[mav];
                                }
                                else
                                {
                                    pid = new Tuple<PID>(new PID(0.55f, 1.0f, 0f, 10, 20, 8, 0));
                                    pids.Add(mav, pid);
                                }
                                var speedp = pid.Item1;

                                // do speed
                                // in m out 0-1
                                speedp.set_input_filter_all((float)dist);

                                // prevent buildup prior to being close
                                if (dist > 55)
                                    speedp.reset_I();

                                // 15m/s demand + pid results
                                float target_speed = (float)MathHelper.constrain(speedp.get_pid(), 15, 35);
                            }
                            else
                            {
                                targetyaw = Leader.cs.yaw;
                                target.Alt = mav.cs.alt;
                                if (err_time <= 5)
                                {
                                    targetspeed = 15;
                                    Console.WriteLine("OUTTIME Speed {1}", targetspeed);
                                }
                                else
                                {
                                    Console.WriteLine("OUTTIME Return");
                                    return;
                                }
                            }

                            Console.WriteLine("sysid {0} - yaw {1} speed {2} dist {3}", mav.sysid, targetyaw, target_speed, dist);

                            //Yaw, Speed, Alt
                            port.sendPacket(new MAVLink.mavlink_command_long_t {
                                            command = (ushort)MAVLink.MAV_CMD.GUIDED_CHANGE_HEADING,
                                            param1 = (float)MAVLink.HEADING_TYPE.COURSE_OVER_GROUND,
                                            param2 = (float)targetyaw,
                                            param3 = 0.0f
                                            },
                                            mav.sysid, mav.compid);

                            port.sendPacket(new MAVLink.mavlink_command_long_t {
                                            command = (ushort)MAVLink.MAV_CMD.GUIDED_CHANGE_SPEED,
                                            param1 = (float)MAVLink.SPEED_TYPE.GROUNDSPEED,
                                            param2 = targetspeed,
                                            param3 = 0.0f
                                            },
                                            mav.sysid, mav.compid);

                            port.sendPacket(new MAVLink.mavlink_command_long_t {
                                            command = (ushort)MAVLink.MAV_CMD.GUIDED_CHANGE_ALTITUDE,
                                            param3 = 0.0f,
                                            param7 = (float)target.Alt,
                                            },
                                            mav.sysid, mav.compid);
                        }
                        // end ArduPlane

                        else
                        {
                            Vector3 vel = new Vector3(Leader.cs.vx, Leader.cs.vy, Leader.cs.vz);

                            // do pos/vel
                            port.setPositionTargetGlobalInt(mav.sysid, mav.compid, true,
                                true, false, false,
                                MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT, target.Lat, target.Lng, target.Alt, vel.x,
                                vel.y, vel.z, 0, 0);

                            // do yaw
                            if (!gimbal)
                            {
                                // within 3 degrees dont send
                                if (Math.Abs(mav.cs.yaw - Leader.cs.yaw) > 3)
                                    port.doCommand(mav.sysid, mav.compid, MAVLink.MAV_CMD.CONDITION_YAW, Leader.cs.yaw,
                                        100.0f, 0, 0, 0, 0, 0, false);
                            }
                            else
                            {
                                // gimbal direction
                                if (Math.Abs(mav.cs.yaw - Leader.cs.yaw) > 3)
                                    port.setMountControl(mav.sysid, mav.compid, 45, 0, Leader.cs.yaw, false);
                            }
                        }

                        //Console.WriteLine("{0} {1} {2} {3}", port.ToString(), target.Lat, target.Lng, target.Alt);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Failed to send command " + mav.ToString() + "\n" + ex.ToString());
                    }

                    a++;
                }
            }
        }

        public bool gimbal { get; set; }
    }

    public class PID
    {
        /*
                    previous_error = 0
                    integral = 0
                    loop:
                    error = setpoint - measured_value
                        integral = integral + error * dt
                    derivative = (error - previous_error) / dt
                        output = Kp * error + Ki * integral + Kd * derivative
                    previous_error = error
                        wait(dt)
                        goto loop*/
        private float _dt;
        private float M_2PI = (float)(Math.PI * 2);
        private float _input;
        private float _derivative;
        private float _kp;
        private float _ki;
        private float _integrator;
        private float _imax;
        private float _kd;
        private float _ff;
        private float _filt_hz = AC_PID_FILT_HZ_DEFAULT;

        const float AC_PID_FILT_HZ_DEFAULT = 20.0f; // default input filter frequency
        const float AC_PID_FILT_HZ_MIN = 0.01f; // minimum input filter frequency

        // Constructor
        public PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff)
        {
            _dt = dt;
            _integrator = 0.0f;
            _input = 0.0f;
            _derivative = 0.0f;

            _kp = initial_p;
            _ki = initial_i;
            _kd = initial_d;
            _imax = Math.Abs(initial_imax);
            filt_hz(initial_filt_hz);
            _ff = initial_ff;

            // reset input filter to first value received
            _flags._reset_filter = true;
        }

        // set_dt - set time step in seconds
        public void set_dt(float dt)
        {
            // set dt and calculate the input filter alpha
            _dt = dt;
        }

        // filt_hz - set input filter hz
        public void filt_hz(float hz)
        {
            _filt_hz = hz;

            // sanity check _filt_hz
            _filt_hz = Math.Max(_filt_hz, AC_PID_FILT_HZ_MIN);
        }

        public void set_input_filter_all(float input)
        {
            // don't process inf or NaN
            if (!isfinite(input))
            {
                return;
            }

            // reset input filter to value received
            if (_flags._reset_filter)
            {
                _flags._reset_filter = false;
                _input = input;
                _derivative = 0.0f;
            }

            // update filter and calculate derivative
            float input_filt_change = get_filt_alpha() * (input - _input);
            _input = _input + input_filt_change;
            if (_dt > 0.0f)
            {
                _derivative = input_filt_change / _dt;
            }
        }

        private bool isfinite(float input)
        {
            return !float.IsInfinity(input);
        }

        public float get_p()
        {
            _pid_info.P = (_input * _kp);
            return _pid_info.P;
        }

        public float get_i()
        {
            if (!is_zero(_ki) && !is_zero(_dt))
            {
                _integrator += ((float)_input * _ki) * _dt;
                if (_integrator < -_imax)
                {
                    _integrator = -_imax;
                }
                else if (_integrator > _imax)
                {
                    _integrator = _imax;
                }

                _pid_info.I = _integrator;
                return _integrator;
            }

            return 0;
        }

        public float get_d()
        {
            // derivative component
            _pid_info.D = (_kd * _derivative);
            return _pid_info.D;
        }

        public float get_ff(float requested_rate)
        {
            _pid_info.FF = (float)requested_rate * _ff;
            return _pid_info.FF;
        }

        public float get_pi()
        {
            return get_p() + get_i();
        }

        public float get_pid()
        {
            return get_p() + get_i() + get_d();
        }

        public void reset_I()
        {
            _integrator = 0;
        }

        public float get_filt_alpha()
        {
            if (is_zero(_filt_hz))
            {
                return 1.0f;
            }

            // calculate alpha
            float rc = 1 / (M_2PI * _filt_hz);
            return _dt / (_dt + rc);
        }

        private bool is_zero(float filt_hz)
        {
            return filt_hz == 0;
        }

        internal class flags
        {
            internal bool _reset_filter;
        }

        flags _flags = new flags();

        pid_info _pid_info = new pid_info();

        internal class pid_info
        {
            internal float P;
            internal float I;
            internal float D;
            internal float FF;
        }
    }


}