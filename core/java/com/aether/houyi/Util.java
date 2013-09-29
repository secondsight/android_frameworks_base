package com.aether.houyi;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

public class Util {
    public static int nextPowerOf2(int n)
    {
        if (n <= 0 || n > (1 << 30))
        {
            return -1;
        }

        n -= 1;
        n |= n >> 16;
        n |= n >> 8;
        n |= n >> 4;
        n |= n >> 2;
        n |= n >> 1;
        return n + 1;
    }
    
    public static void glhPerspectivef(float[] matrix, float fovyInDegrees,
            float aspectRatio, float znear, float zfar) {
        float ymax, xmax;
        // float temp, temp2, temp3, temp4;
        ymax = znear * (float) Math.tan(fovyInDegrees * Math.PI / 360.0);
        // ymin = -ymax;
        // xmin = -ymax * aspectRatio;
        xmax = ymax * aspectRatio;
        glhFrustumf2(matrix, -xmax, xmax, -ymax, ymax, znear, zfar);
    }

    public static void glhFrustumf2(float[] matrix, float left, float right,
            float bottom, float top, float znear, float zfar) {
        float temp, temp2, temp3, temp4;
        temp = 2.0f * znear;
        temp2 = right - left;
        temp3 = top - bottom;
        temp4 = zfar - znear;
        matrix[0] = temp / temp2;
        matrix[1] = 0.0f;
        matrix[2] = 0.0f;
        matrix[3] = 0.0f;
        matrix[4] = 0.0f;
        matrix[5] = temp / temp3;
        matrix[6] = 0.0f;
        matrix[7] = 0.0f;
        matrix[8] = (right + left) / temp2;
        matrix[9] = (top + bottom) / temp3;
        matrix[10] = (-zfar - znear) / temp4;
        matrix[11] = -1.0f;
        matrix[12] = 0.0f;
        matrix[13] = 0.0f;
        matrix[14] = (-temp * zfar) / temp4;
        matrix[15] = 0.0f;
    }
    
    public static FloatBuffer getFloatBufferFromFloatArray(float array[]) {
        ByteBuffer tempBuffer = ByteBuffer.allocateDirect(array.length * 4);
        tempBuffer.order(ByteOrder.nativeOrder());
        FloatBuffer buffer = tempBuffer.asFloatBuffer();
        buffer.put(array);
        buffer.position(0);
        return buffer;
    }
    
    public static <T extends Comparable<T>> T clamp(T val, T min, T max) {
        if (val.compareTo(min) < 0) return min;
        else if(val.compareTo(max) > 0) return max;
        else return val;
    }
}
