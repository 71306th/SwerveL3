package frc.FSLib.math;

public class PolynomialRegression {

    // 已知數值(已測試)
    private static final double[] tzKnownValues = {2.25, 2.5, 3, 3.35, 3.5, 3.75, 4};
    private static final double[] degKnownValues = {-0.1789, -0.1655, -0.1574, -0.1543, -0.1496, -0.1482, -0.1438};

    public static void main(String[] args) {
        // 測試輸入(可修改下方數值)
        double tzInput = 3.4;
        double degOutput = predictDeg(tzInput);
        System.out.println("預測的deg值為: " + degOutput);
    }

    public static double predictDeg(double tzInput) {
        // 計算二次多項式的係數
        double[] coefficients = fitPolynomial(tzKnownValues, degKnownValues, 2);
        // 預測deg值
        return evaluatePolynomial(coefficients, tzInput);
    }

    // 使用最小二乘法擬合多項式
    private static double[] fitPolynomial(double[] x, double[] y, int degree) {
        int n = x.length;
        double[][] matrix = new double[degree + 1][degree + 2];
        double[] result = new double[degree + 1];

        for (int i = 0; i <= degree; i++) {
            for (int j = 0; j <= degree; j++) {
                matrix[i][j] = 0;
                for (int k = 0; k < n; k++) {
                    matrix[i][j] += Math.pow(x[k], i + j);
                }
            }
            matrix[i][degree + 1] = 0;
            for (int k = 0; k < n; k++) {
                matrix[i][degree + 1] += Math.pow(x[k], i) * y[k];
            }
        }

        // 高斯消去法
        for (int i = 0; i <= degree; i++) {
            for (int k = i + 1; k <= degree; k++) {
                if (Math.abs(matrix[i][i]) < Math.abs(matrix[k][i])) {
                    double[] temp = matrix[i];
                    matrix[i] = matrix[k];
                    matrix[k] = temp;
                }
            }
            for (int k = i + 1; k <= degree; k++) {
                double t = matrix[k][i] / matrix[i][i];
                for (int j = 0; j <= degree + 1; j++) {
                    matrix[k][j] -= t * matrix[i][j];
                }
            }
        }
        for (int i = degree; i >= 0; i--) {
            result[i] = matrix[i][degree + 1] / matrix[i][i];
            for (int k = 0; k < i; k++) {
                matrix[k][degree + 1] -= matrix[k][i] * result[i];
            }
        }

        return result;
    }

    // 計算多項式的值
    private static double evaluatePolynomial(double[] coefficients, double x) {
        double result = 0;
        for (int i = coefficients.length - 1; i >= 0; i--) {
            result = result * x + coefficients[i];
        }
        return result;
    }
}