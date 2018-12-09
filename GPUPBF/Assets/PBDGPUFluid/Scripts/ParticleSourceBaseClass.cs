using System;
using System.Collections.Generic;
using UnityEngine;

namespace PBFluid
{
    // the base class that defines a set of particles
    public abstract class ParticleSource
    {
        //number of the particles
        public int NumParticles { get { return Positions.Count; } }
        //particle positions
        public IList<Vector3> Positions { get; protected set; }
        //the interval(or stride, spacing), defining how the space is divided into particles
        public float Interval { get; private set; }

        public float HalfInterval { get { return Interval * 0.5f; } }

        public ParticleSource(float interval)
        {
            Interval = interval;
        }

    }

}