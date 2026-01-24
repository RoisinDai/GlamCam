Shader "Custom/DepthMask"
{
    SubShader
    {
        // Render before regular opaque objects (Geometry-1)
        // to prime the depth buffer early.
        Tags { "Queue" = "Geometry-1" }

        Pass
        {
            // Do not write to the Red, Green, Blue, or Alpha channels
            ColorMask 0 
            
            // Do write to the Depth buffer
            ZWrite On
        }
    }
}