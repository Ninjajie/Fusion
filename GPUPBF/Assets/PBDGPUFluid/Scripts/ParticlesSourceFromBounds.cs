using System;
using System.Collections.Generic;
using UnityEngine;

namespace PBFluid
{

    public class ParticlesFromBounds : ParticleSource
    {
        //the outmost bound
        public Bounds Bounds { get; private set; }
        //for fluid body, need to create particles for space in the middle
        //but for boundaries not
        //the fluid boundary is defined by at least two Bounds: inner and outer
        public List<Bounds> BoundsForExclusion { get; private set; }

        public ParticlesFromBounds(float Interval, Bounds bounds) : base(Interval)
        {
            Bounds = bounds;
            BoundsForExclusion = new List<Bounds>();
            CreateParticles();
        }

        public ParticlesFromBounds(float Interval, Bounds bounds, Bounds boundsForExclusion) : base(Interval)
        {
            Bounds = bounds;
            BoundsForExclusion = new List<Bounds>();
            BoundsForExclusion.Add(boundsForExclusion);
            CreateParticles();
        }

        private void CreateParticles()
        {

            int numX = (int)((Bounds.size.x + HalfInterval) / Interval);
            int numY = (int)((Bounds.size.y + HalfInterval) / Interval);
            int numZ = (int)((Bounds.size.z + HalfInterval) / Interval);

            Positions = new List<Vector3>();

            for (int z = 0; z < numZ; z++)
            {
                for (int y = 0; y < numY; y++)
                {
                    for (int x = 0; x < numX; x++)
                    {
                        Vector3 pos = new Vector3();
                        pos.x = Interval * x + Bounds.min.x + HalfInterval;
                        pos.y = Interval * y + Bounds.min.y + HalfInterval;
                        pos.z = Interval * z + Bounds.min.z + HalfInterval;

                        bool exclude = false;
                        for (int i = 0; i < BoundsForExclusion.Count; i++)
                        {
                            if (BoundsForExclusion[i].Contains(pos))
                            {
                                exclude = true;
                                break;
                            }
                        }

                        if (!exclude)
                            Positions.Add(pos);
                    }
                }
            }

        }

    }

}