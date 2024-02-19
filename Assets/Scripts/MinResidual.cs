
using UnityEngine;


public class MinResidual
{
    /// <summary>
    /// 残差平方和
    /// </summary>
    /// <param name="observeTransform"></param>
    /// <param name="actualTransform"></param>
    /// <returns></returns>
    public static float GetResidualSumSquares(Transform[] observeTransform, Transform[] actualTransform)
    {
        var rss = 0f;
        if (observeTransform.Length != actualTransform.Length) Debug.LogErrorFormat("长度不一致 observeLenght:{0} actualLenght{1}", observeTransform.Length, actualTransform.Length);
        for (int i = 0; i < observeTransform.Length; i++)
        {
            rss += GetDistance(observeTransform[i].position, actualTransform[i].position);
        }
        return rss;
    }

    public static float GetResidualSumSquares(Vector3[] observeTransform, Vector3[] actualTransform)
    {
        var rss = 0f;
        if (observeTransform.Length != actualTransform.Length) Debug.LogErrorFormat("长度不一致 observeLenght:{0} actualLenght{1}", observeTransform.Length, actualTransform.Length);
        for (int i = 0; i < observeTransform.Length; i++)
        {
            rss += GetDistance(observeTransform[i], actualTransform[i]);
        }
        return rss;
    }

    /// <summary>
    /// 均方误差
    /// </summary>
    /// <param name="observeTransform"></param>
    /// <param name="actualTransform"></param>
    /// <returns></returns>
    public static float GetMeanSquareError(Transform[] observeTransform, Transform[] actualTransform)
    {
        var mse = GetResidualSumSquares(observeTransform, actualTransform) / observeTransform.Length;
        return mse;
    }

    public static float GetMeanSquareError(Vector3[] observeTransform, Vector3[] actualTransform)
    {
        var mse = GetResidualSumSquares(observeTransform, actualTransform) / observeTransform.Length;
        return mse;
    }

    private static float GetDistance(Vector3 a, Vector3 b)
    {
        var distance = Vector3.Distance(a, b);
        return Mathf.Pow(distance, 2);
    }
}
