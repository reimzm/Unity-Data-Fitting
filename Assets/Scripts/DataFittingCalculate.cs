using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Linq;


public class DataFittingCalculate : MonoBehaviour
{
    /// <summary>
    /// 拟合对象
    /// </summary>
    public Transform DataFitting;

    /// <summary>
    /// 实际模型测量父物体
    /// </summary>
    public Transform ActualParent;

    /// <summary>
    /// 观测测量父物体（ART测试的参考数据）
    /// </summary>
    public Transform ObservedParent;

    /// <summary>
    /// 实际模型测量数据组
    /// </summary>
    public Transform[] ActualPostions;
    /// <summary>
    /// 观测测量数据组（ART测试的参考数据）
    /// </summary>
    public Transform[] ObservedPostions;

    /// <summary>
    /// 当前观测数据和实际数据的残差平方和
    /// </summary>
    public float RSS;

    /// <summary>
    /// 当最小残差平方和小于这个值时便记录当前数据
    /// </summary>
    public float Min = 0.0001f;

    public bool IsStartCalculate = true;
    // Start is called before the first frame update
    void Start()
    {

        if (ActualPostions.Length == 0 || ActualPostions == null && ActualParent != null)
        {
            ActualPostions = new Transform[ActualParent.childCount];
            for (int i = 0; i < ActualPostions.Length; i++)
            {
                ActualPostions[i] = ActualParent.GetChild(i);
            }
        }

        if (ObservedPostions.Length == 0 || ObservedPostions == null && ObservedParent != null)
        {
            ObservedPostions = new Transform[ObservedParent.childCount];
            for (int i = 0; i < ObservedPostions.Length; i++)
            {
                ObservedPostions[i] = ObservedParent.GetChild(i);
            }
        }

        if (DataFitting == null)
        {
            DataFitting = ActualParent;
        }
    }

    private void Update()
    {
        if (IsStartCalculate)
        {
            ///首先计算出两组数据的旋转矩阵和平移向量
            var TR = OptimalRotationTranslation(ConvertTransformArrayToMatrix(ObservedPostions), ConvertTransformArrayToMatrix(ActualPostions));

            //将旋转矩阵和平移向量转换成unity 能够使用的vector3
            var eulerAngles = MatrixToEulerAngles(TR.Item2);
            var TranslationVector = ConvertMathNetVectorToVector3(TR.Item1);

            ///拟合的旋转矩阵之间赋值，偏移向量相加
            DataFitting.eulerAngles = eulerAngles;
            DataFitting.position += TranslationVector;

            ///计算最小残差平方和
            RSS = MinResidual.GetResidualSumSquares(ObservedPostions, ActualPostions);
            Debug.Log(RSS);
            if (RSS < Min)
            {
                IsStartCalculate = false;
            }
        }
    }


    /// <summary>
    /// 计算观测数据和实际数据的旋转矩阵和平移向量
    /// </summary>
    /// <param name="observedPoints">观测数据组</param>
    /// <param name="actualPoints">实际数据组</param>
    /// <returns></returns>
    public static (Vector<double>, Matrix<double>) OptimalRotationTranslation(Matrix<double> observedPoints, Matrix<double> actualPoints)
    {

        ///计算质心
        var centroidOriginal = ColumnWiseMean(actualPoints);
        var centroidTransformed = ColumnWiseMean(observedPoints);

        ///中心化点集
        var originalPointsCentered = CenterData(actualPoints, centroidOriginal);
        var transformedPointsCentered = CenterData(observedPoints, centroidTransformed);

        ///计算协方差矩阵和旋转矩阵
        var H = originalPointsCentered.TransposeThisAndMultiply(transformedPointsCentered);

        ///使用SVD计算旋转矩阵
        var svd = H.Svd(true);
        var U = svd.U;
        var Vt = svd.VT;
        var rotationMatrix = Vt.TransposeThisAndMultiply(U.Transpose());

        ///确保旋转矩阵是正交的
        if (rotationMatrix.Determinant() < 0)
        {
            Vt.SetRow(2, Vt.Row(2).Multiply(-1));
            rotationMatrix = Vt.TransposeThisAndMultiply(U.Transpose());
        }

        ///计算平移向量
        var translationVector = -rotationMatrix.Multiply(centroidOriginal) + centroidTransformed;

        return (translationVector, rotationMatrix);


    }

    /// <summary>
    /// 计算质心
    /// </summary>
    /// <param name="matrix"></param>
    /// <returns></returns>
    public static Vector<double> ColumnWiseMean(Matrix<double> matrix)
    {
        return DenseVector.OfEnumerable(Enumerable.Range(0, matrix.ColumnCount).Select(i => matrix.Column(i).Average()));
    }

    /// <summary>
    /// 中心化点集
    /// </summary>
    /// <param name="data"></param>
    /// <param name="centroid"></param>
    /// <returns></returns>
    public static Matrix<double> CenterData(Matrix<double> data, Vector<double> centroid)
    {
        return DenseMatrix.OfRowVectors(data.EnumerateRows().Select(row => row - centroid));
    }

    /// <summary>
    /// 3x3矩阵转换成欧拉角
    /// </summary>
    /// <param name="matrix"></param>
    /// <returns></returns>
    public static Vector3 MatrixToEulerAngles(Matrix<double> matrix)
    {
        // 假设matrix是一个3x3的旋转矩阵
        double m11 = matrix[0, 0], m12 = matrix[0, 1], m13 = matrix[0, 2];
        double m21 = matrix[1, 0], m22 = matrix[1, 1], m23 = matrix[1, 2];
        double m31 = matrix[2, 0], m32 = matrix[2, 1], m33 = matrix[2, 2];

        // 计算四元数的w, x, y, z分量
        double w = Math.Sqrt(1.0 + m11 + m22 + m33) / 2.0;
        double w4 = (4.0 * w);
        double x = (m32 - m23) / w4;
        double y = (m13 - m31) / w4;
        double z = (m21 - m12) / w4;

        // 使用Unity的Quaternion构造器
        Quaternion q = new Quaternion((float)x, (float)y, (float)z, (float)w);

        // 转换为欧拉角
        return q.eulerAngles;
    }

    /// <summary>
    /// transform下的数据位置转换成用于计算的double类型数据组（不用看作矩阵）
    /// </summary>
    /// <param name="transforms"></param>
    /// <returns></returns>
    public static Matrix<double> ConvertTransformArrayToMatrix(Transform[] transforms)
    {
        List<double[]> list_dataArray = new List<double[]>();
        for (int i = 0; i < transforms.Length; i++)
        {
            double[] v3 = new double[] { transforms[i].position.x, transforms[i].position.y, transforms[i].position.z };
            list_dataArray.Add(v3);
        }
        var dataArray = list_dataArray.ToArray();
        // 假设 dataArray 是非空的且所有内部数组长度相等
        int rows = dataArray.Length;
        int cols = dataArray[0].Length;

        // 使用 DenseMatrix 的 Create 方法来创建一个新的矩阵
        var matrix = DenseMatrix.Create(rows, cols, 0);

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                // 将 float 值转换为 double 并赋值
                matrix[i, j] = dataArray[i][j];
            }
        }

        return matrix;
    }

    /// <summary>
    /// 将MathNet库下的向量转换成unity Vector3
    /// </summary>
    /// <param name="mathNetVector"></param>
    /// <returns></returns>
    public static Vector3 ConvertMathNetVectorToVector3(Vector<double> mathNetVector)
    {
        return new Vector3((float)mathNetVector[0], (float)mathNetVector[1], (float)mathNetVector[2]);
    }

}