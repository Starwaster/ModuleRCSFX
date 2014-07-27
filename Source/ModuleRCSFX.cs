using System;
using System.Collections.Generic;
using UnityEngine;
using KSP;

public class ModuleRCSFX : ModuleRCS
{
    
    [KSPField()]
    public bool useZaxis = false;
    [KSPField]
    public bool enableYaw = true;
    [KSPField]
    public bool enablePitch = true;
    [KSPField]
    public bool enableRoll = true;

    [KSPField]
    public bool enableX = true;
    [KSPField]
    public bool enableY = true;
    [KSPField]
    public bool enableZ = true;

    [KSPField]
    public bool useThrottle = false;


    public float mixtureFactor;

    public override void OnLoad(ConfigNode node)
    {
        if (!node.HasNode("PROPELLANT"))
        {
            ConfigNode c = new ConfigNode();
            c.SetValue("name", resourceName);
            c.SetValue("ratio", "1.0");
            node.AddNode(c);
        }
        base.SetupPropellant(node);
        G = 9.80665f;
    }

    public override void OnStart(StartState state)
    {
        base.OnStart(state);
    }

    public float getMaxFuelFlow(Propellant p)
    {
        return p.ratio * this.thrusterPower / (atmosphereCurve.Evaluate(0f) * 9.82f) / this.resourceMass;
    }

    public override string GetInfo()
    {
        string info = "<b>Thruster Power: </b>" + thrusterPower.ToString("F") + "\n";
        info += "<b>Thruster ISP: </b>" + atmosphereCurve.Evaluate(1f).ToString("F0")
              + " (ASL) - " + atmosphereCurve.Evaluate(0f).ToString("F0") + "(Vac)\n";
        info += "<color=#99ff00ff><b>Requires:</b></color> \n";
        foreach (Propellant p in propellants)
            info += "<b>" + p.name + ": </b>" + getMaxFuelFlow(p).ToString("F4") + "/sec. Max. \n";
        return info;
    }

    Vector3 inputLinear;
    Vector3 inputAngular;
    bool precision;

    new public void Update()
    {
        if (this.part.vessel == null)
            return;

        inputLinear = vessel.ReferenceTransform.rotation * new Vector3(enableX ? vessel.ctrlState.X : 0f, enableZ ? vessel.ctrlState.Z : 0f, enableY ? vessel.ctrlState.Y : 0f);
        inputAngular = vessel.ReferenceTransform.rotation * new Vector3(enablePitch ? vessel.ctrlState.pitch : 0f, enableRoll ? vessel.ctrlState.roll : 0f, enableYaw ? vessel.ctrlState.yaw : 0);
        if (useThrottle)
        {
            inputLinear.y -= vessel.ctrlState.mainThrottle;
            inputLinear.y = Mathf.Clamp(inputLinear.y, - 1f, 1f);
        }
        precision = FlightInputHandler.fetch.precisionMode;
    }

    new public void FixedUpdate()
    {
        if (HighLogic.LoadedSceneIsEditor)
            return;

        if (TimeWarp.CurrentRate > 1.0f && TimeWarp.WarpMode == TimeWarp.Modes.HIGH)
        {
            foreach (FXGroup fx in thrusterFX)
            {
                fx.setActive(false);
                fx.Power = 0f;
            }
            return;
        }

        bool success = false;
        realISP = atmosphereCurve.Evaluate((float)vessel.staticPressure);
        thrustForces.Clear();
        if (isEnabled && part.isControllable)
        {
            if (vessel.ActionGroups[KSPActionGroup.RCS])
            {
                Vector3 CoM = vessel.CoM + vessel.rb_velocity * Time.deltaTime;

                float effectPower = 0f;
                for (int i = 0; i < thrusterTransforms.Count; i++)
                {
                    if (thrusterTransforms[i].position != Vector3.zero)
                    {
                        Vector3 position = thrusterTransforms[i].transform.position;
                        Vector3 torque = Vector3.Cross(inputAngular, (position - (CoM + vessel.rb_velocity * Time.deltaTime)).normalized);
                        Vector3 thruster;
                        if (useZaxis)
                            thruster = thrusterTransforms[i].forward;
                        else
                            thruster = thrusterTransforms[i].up;
                        float thrust = Mathf.Max(Vector3.Dot(thruster, torque), 0f);
                        thrust += Mathf.Max(Vector3.Dot(thruster, inputLinear), 0f);
                        if (thrust > 0.0001f)
                        {
                            if (precision)
                            {
                                float arm = GetLeverDistance(-thruster, CoM);
                                if (arm > 1.0f)
                                    thrust = thrust / arm;
                            }
                            UpdatePropellantStatus();

                            thrust = CalculateThrust(thrust, out success);
                            thrustForces.Add(thrust);
                            if (success)
                            {
                                if (!isJustForShow)
                                {
                                    Vector3 force = (-1 * thrust) * thruster;

                                    part.Rigidbody.AddForceAtPosition(force, position, ForceMode.Force);
                                }

                                thrusterFX[i].Power = Mathf.Clamp(thrust / thrusterPower, 0.1f, 1f);
                                if (effectPower < thrusterFX[i].Power)
                                    effectPower = thrusterFX[i].Power;
                                thrusterFX[i].setActive(thrust > 0f);
                            }
                        }
                    }
                }
            }
        }
        if (!success)
        {
            foreach (FXGroup fx in thrusterFX)
            {
                fx.setActive(false);
                fx.Power = 0f;
            }
        }

    }

    private void UpdatePropellantStatus()
    {
        if ((object)propellants != null)
            foreach (Propellant p in propellants)
                p.UpdateConnectedResources(part);
    }

}
