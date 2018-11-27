using UnityEngine;

public class SmoothRandom
{
    public static Vector3 GetVector3(float speed)
    {
        float time = Time.time * 0.01F * speed;
        return new Vector3(Get().HybridMultifractal(time, 15.73F, 0.58F), Get().HybridMultifractal(time, 63.94F, 0.58F),
            Get().HybridMultifractal(time, 0.2F, 0.58F));
    }

    public static float Get(float speed)
    {
        float time = Time.time * 0.01F * speed;
        return Get().HybridMultifractal(time * 0.01F, 15.7F, 0.65F);
    }

    private static FractalNoise Get()
    {
        if (s_Noise == null)
        {
            s_Noise = new FractalNoise(1.27F, 2.04F, 8.36F);
        }
        return s_Noise;
    }

    private static FractalNoise s_Noise;
}