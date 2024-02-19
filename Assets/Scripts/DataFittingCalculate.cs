using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Linq;


public class DataFittingCalculate : MonoBehaviour
{
    /// <summary>
    /// ��϶���
    /// </summary>
    public Transform DataFitting;

    /// <summary>
    /// ʵ��ģ�Ͳ���������
    /// </summary>
    public Transform ActualParent;

    /// <summary>
    /// �۲���������壨ART���ԵĲο����ݣ�
    /// </summary>
    public Transform ObservedParent;

    /// <summary>
    /// ʵ��ģ�Ͳ���������
    /// </summary>
    public Transform[] ActualPostions;
    /// <summary>
    /// �۲���������飨ART���ԵĲο����ݣ�
    /// </summary>
    public Transform[] ObservedPostions;

    /// <summary>
    /// ��ǰ�۲����ݺ�ʵ�����ݵĲв�ƽ����
    /// </summary>
    public float RSS;

    /// <summary>
    /// ����С�в�ƽ����С�����ֵʱ���¼��ǰ����
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
            ///���ȼ�����������ݵ���ת�����ƽ������
            var TR = OptimalRotationTranslation(ConvertTransformArrayToMatrix(ObservedPostions), ConvertTransformArrayToMatrix(ActualPostions));

            //����ת�����ƽ������ת����unity �ܹ�ʹ�õ�vector3
            var eulerAngles = MatrixToEulerAngles(TR.Item2);
            var TranslationVector = ConvertMathNetVectorToVector3(TR.Item1);

            ///��ϵ���ת����֮�丳ֵ��ƫ���������
            DataFitting.eulerAngles = eulerAngles;
            DataFitting.position += TranslationVector;

            ///������С�в�ƽ����
            RSS = MinResidual.GetResidualSumSquares(ObservedPostions, ActualPostions);
            Debug.Log(RSS);
            if (RSS < Min)
            {
                IsStartCalculate = false;
            }
        }
    }


    /// <summary>
    /// ����۲����ݺ�ʵ�����ݵ���ת�����ƽ������
    /// </summary>
    /// <param name="observedPoints">�۲�������</param>
    /// <param name="actualPoints">ʵ��������</param>
    /// <returns></returns>
    public static (Vector<double>, Matrix<double>) OptimalRotationTranslation(Matrix<double> observedPoints, Matrix<double> actualPoints)
    {

        ///��������
        var centroidOriginal = ColumnWiseMean(actualPoints);
        var centroidTransformed = ColumnWiseMean(observedPoints);

        ///���Ļ��㼯
        var originalPointsCentered = CenterData(actualPoints, centroidOriginal);
        var transformedPointsCentered = CenterData(observedPoints, centroidTransformed);

        ///����Э����������ת����
        var H = originalPointsCentered.TransposeThisAndMultiply(transformedPointsCentered);

        ///ʹ��SVD������ת����
        var svd = H.Svd(true);
        var U = svd.U;
        var Vt = svd.VT;
        var rotationMatrix = Vt.TransposeThisAndMultiply(U.Transpose());

        ///ȷ����ת������������
        if (rotationMatrix.Determinant() < 0)
        {
            Vt.SetRow(2, Vt.Row(2).Multiply(-1));
            rotationMatrix = Vt.TransposeThisAndMultiply(U.Transpose());
        }

        ///����ƽ������
        var translationVector = -rotationMatrix.Multiply(centroidOriginal) + centroidTransformed;

        return (translationVector, rotationMatrix);


    }

    /// <summary>
    /// ��������
    /// </summary>
    /// <param name="matrix"></param>
    /// <returns></returns>
    public static Vector<double> ColumnWiseMean(Matrix<double> matrix)
    {
        return DenseVector.OfEnumerable(Enumerable.Range(0, matrix.ColumnCount).Select(i => matrix.Column(i).Average()));
    }

    /// <summary>
    /// ���Ļ��㼯
    /// </summary>
    /// <param name="data"></param>
    /// <param name="centroid"></param>
    /// <returns></returns>
    public static Matrix<double> CenterData(Matrix<double> data, Vector<double> centroid)
    {
        return DenseMatrix.OfRowVectors(data.EnumerateRows().Select(row => row - centroid));
    }

    /// <summary>
    /// 3x3����ת����ŷ����
    /// </summary>
    /// <param name="matrix"></param>
    /// <returns></returns>
    public static Vector3 MatrixToEulerAngles(Matrix<double> matrix)
    {
        // ����matrix��һ��3x3����ת����
        double m11 = matrix[0, 0], m12 = matrix[0, 1], m13 = matrix[0, 2];
        double m21 = matrix[1, 0], m22 = matrix[1, 1], m23 = matrix[1, 2];
        double m31 = matrix[2, 0], m32 = matrix[2, 1], m33 = matrix[2, 2];

        // ������Ԫ����w, x, y, z����
        double w = Math.Sqrt(1.0 + m11 + m22 + m33) / 2.0;
        double w4 = (4.0 * w);
        double x = (m32 - m23) / w4;
        double y = (m13 - m31) / w4;
        double z = (m21 - m12) / w4;

        // ʹ��Unity��Quaternion������
        Quaternion q = new Quaternion((float)x, (float)y, (float)z, (float)w);

        // ת��Ϊŷ����
        return q.eulerAngles;
    }

    /// <summary>
    /// transform�µ�����λ��ת�������ڼ����double���������飨���ÿ�������
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
        // ���� dataArray �Ƿǿյ��������ڲ����鳤�����
        int rows = dataArray.Length;
        int cols = dataArray[0].Length;

        // ʹ�� DenseMatrix �� Create ����������һ���µľ���
        var matrix = DenseMatrix.Create(rows, cols, 0);

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                // �� float ֵת��Ϊ double ����ֵ
                matrix[i, j] = dataArray[i][j];
            }
        }

        return matrix;
    }

    /// <summary>
    /// ��MathNet���µ�����ת����unity Vector3
    /// </summary>
    /// <param name="mathNetVector"></param>
    /// <returns></returns>
    public static Vector3 ConvertMathNetVectorToVector3(Vector<double> mathNetVector)
    {
        return new Vector3((float)mathNetVector[0], (float)mathNetVector[1], (float)mathNetVector[2]);
    }

}