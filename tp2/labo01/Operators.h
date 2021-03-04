#pragma once

/**
 * @file Operators.h
 *
 * @brief Opérateurs arithmétiques pour les matrices et les vecteurs.
 *
 * Nom: Richard Layhout Lao
 * Code permanent : LAO19089501
 * Email : richard-layhout.lao.1@ens.etsmtl.ca
 *
 */

#include "Matrix.h"
#include "Vector.h"

 /**
  * Implémentation de divers opérateurs arithmétiques pour les matrices et les
  * vecteurs.
  */
namespace gti320 {

    /**
     * Multiplication : Matrix * Matrix (générique)
     */
    template <typename _Scalar, int RowsA, int ColsA, int StorageA, int RowsB, int ColsB, int StorageB>
    Matrix<_Scalar, RowsA, ColsB> operator*(const Matrix<_Scalar, RowsA, ColsA, StorageA>& A, const Matrix<_Scalar, RowsB, ColsB, StorageB>& B)
    {

        /*
           CODE EMPRUNTÉ : Provient de l'implémentation naive dans la class main.cpp
        */

        assert(A.cols() == B.rows());
        Matrix<_Scalar, RowsA, ColsB> product(A.rows(), B.cols());
        product.setZero();
        for (int i = 0; i < A.rows(); ++i)
        {
            for (int j = 0; j < B.cols(); ++j)
            {
                for (int k = 0; k < A.cols(); ++k)
                {
                    product(i, j) += A(i, k) * B(k, j);
                }
            }
        }
        return product;
    }

    /**
     * Multiplication : Matrix(colonnes) * Matrix
     *
     * Spécialisation de l'opérateur de multiplication pour le cas où la matrice
     * de gauche utilise un stockage par colonnes.
     */
    template <typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic> operator*(const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& B)
    {
        /*
           CODE EMPRUNTÉ : Provient de l'implémentation naive dans la class main.cpp
        */
        assert(A.cols() == B.rows());

        Matrix<_Scalar, Dynamic, Dynamic> matrice(A.rows(), B.cols());
        matrice.setZero();
        for (int i = 0; i < A.rows(); ++i)
        {
            for (int j = 0; j < B.cols(); ++j)
            {
                for (int k = 0; k < A.cols(); ++k)
                {
                    matrice(i, j) += A(i, k) * B(k, j);
                }
            }
        }
        return matrice;
    }

    /**
     * Multiplication : Matrix(ligne) * Matrix(colonne)
     *
     * Spécialisation de l'opérateur de multiplication pour le cas où la matrice
     * de gauche utilise un stockage par lignes et celle de gauche un stockage
     * par colonnes.
     */
    template <typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic> operator*(const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& B)
    {
        /*
           CODE EMPRUNTÉ : Provient de l'implémentation naive dans la class main.cpp
        */
        assert(A.cols() == B.rows());
        Matrix<_Scalar, Dynamic, Dynamic> matrice(A.rows(), B.cols());
        matrice.setZero();
        for (int i = 0; i < A.rows(); ++i)
        {
            for (int j = 0; j < B.cols(); ++j)
            {
                for (int k = 0; k < A.cols(); ++k)
                {
                    matrice(i, j) += A(i, k) * B(k, j);
                }
            }
        }
        return matrice;
    }


    /**
     * Addition : Matrix + Matrix (générique)
     */
    template <typename _Scalar, int Rows, int Cols, int StorageA, int StorageB>
    Matrix<_Scalar, Rows, Cols> operator+(const Matrix<_Scalar, Rows, Cols, StorageA>& A, const Matrix<_Scalar, Rows, Cols, StorageB>& B)
    {
        assert(A.cols() == B.cols() && A.rows() == B.rows());

        Matrix<_Scalar, Dynamic, Dynamic> matrice(B.rows(), B.cols());

        for (int i = 0; i < A.rows(); i++) {
            for (int j = 0; j < A.cols(); j++) {
                matrice(i,j) = A(i,j) + B(i,j);
            }
        }
        return matrice;
    }

    /**
     * Addition : Matrix(colonne) + Matrix(colonne)                                                
     *
     * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
     * sont stockées par colonnes.
     */
    template <typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic> operator+(const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& B)
    {
        assert(B.rows() == A.rows() && B.cols() == A.cols());

        Matrix<_Scalar, Dynamic, Dynamic> matrice(B.rows(), B.cols());
        for (int i = 0; i < A.rows(); i++){
            for (int j = 0; j < A.cols(); j++){
                matrice(j, i) = A(j, i) + B(j, i);
            }
        }

        return matrice;
    }

    /**
     * Addition : Matrix(ligne) + Matrix(ligne) a revoir
     *
     * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
     * sont stockées par lignes.
     */
    template <typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic, RowStorage> operator+(const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& B)
    {
        assert(A.cols() == B.cols() && A.rows() == B.rows());

        Matrix<_Scalar, Dynamic, Dynamic, RowStorage> matrice(B.rows(), B.cols());

        for (int i = 0; i < A.rows(); i++){
            for (int j = 0; j < A.cols(); j++){
                matrice(i, j) = A(i, j) + B(i, j);
            }
        }

        return matrice;
    }

    /**
     * Multiplication  : Scalaire * Matrix(colonne)
     *
     * Spécialisation de l'opérateur de multiplication par un scalaire pour le
     * cas d'une matrice stockée par colonnes.
     */
    template <typename _Scalar, int _Rows, int _Cols>
    Matrix<_Scalar, _Rows, _Cols, ColumnStorage> operator*(const _Scalar& a, const Matrix<_Scalar, _Rows, _Cols, ColumnStorage>& A)
    {
        Matrix<_Scalar, Dynamic, Dynamic> matrice(A.rows(), A.cols());

        for (int i = 0; i < A.rows(); i++){
            for (int j = 0; j < A.cols(); j++){
                matrice(j,i) = A(j,i) * a;
            }
        }

        return matrice;
    }

    /**
     * Multiplication  : Scalaire * Matrix(ligne)
     *
     * Spécialisation de l'opérateur de multiplication par un scalaire pour le
     * cas d'une matrice stockée par lignes.
     */
    template <typename _Scalar, int _Rows, int _Cols>
    Matrix<_Scalar, _Rows, _Cols, RowStorage> operator*(const _Scalar& a, const Matrix<_Scalar, _Rows, _Cols, RowStorage>& A)
    {
        Matrix<_Scalar, -1, -1, RowStorage> matrice(A.rows(), A.rows());

        for (int i = 0; i < A.rows(); i++){
            for (int j = 0; j < A.cols(); j++){
                matrice(i,j) = A(i,j) * a;
            }
        }

        return matrice;
    }

    /**
     * Multiplication : Matrice(ligne) * Vecteur
     *
     * Spécialisation de l'opérateur de multiplication matrice*vecteur pour le
     * cas où la matrice est représentée par lignes.
     */
    template <typename _Scalar, int _Rows, int _Cols>
    Vector<_Scalar, _Rows> operator*(const Matrix<_Scalar, _Rows, _Cols, RowStorage>& A, const Vector<_Scalar, _Cols>& v)
    {
        double total;
        Vector<_Scalar, _Rows> vecteur(A.rows());
        vecteur.setZero();
        for (int i = 0; i < A.rows(); i++)
        {
            total = 0;
            for (int j= 0; j < A.cols();j++)
            {
                vecteur(i) += A(i, j) * v(j);
            }
        }

        return vecteur;
    }

    /**
     * Multiplication : Matrice(colonne) * Vecteur                                                          
     *
     * Spécialisation de l'opérateur de multiplication matrice*vecteur pour le
     * cas où la matrice est représentée par colonnes.
     */
    template <typename _Scalar, int _Rows, int _Cols>
    Vector<_Scalar, _Rows> operator*(const Matrix<_Scalar, _Rows, _Cols, ColumnStorage>& A, const Vector<_Scalar, _Cols>& v)
    {
        double total;
        Vector<_Scalar, _Rows> vecteur(A.rows());
        vecteur.setZero();
        for (int j = 0; j < A.cols(); j++)
        {
            total = 0;
            for (int i = 0; i < A.rows(); i++)
            {
                vecteur(i) += A(i, j) * v(j);
            }
      
        }

        return vecteur;
    }

    /**
     * Multiplication : Scalaire * Vecteur                                                                       
     * 
     */
    template <typename _Scalar, int _Rows>
    Vector<_Scalar, _Rows> operator*(const _Scalar& a, const Vector<_Scalar, _Rows>& v)
    {
        Vector<_Scalar, _Rows> vecteur(v.rows());

        for (int i = 0; i < v.rows(); i++) {
            vecteur(i) = v(i) * a;
        }

        return vecteur;
    }


    /**
     * Addition : Vecteur + Vecteur
     */
    template <typename _Scalar, int _RowsA, int _RowsB>
    Vector<_Scalar, _RowsA> operator+(const Vector<_Scalar, _RowsA>& a, const Vector<_Scalar, _RowsB>& b)
    {
        assert(a.size() == b.size());
        Vector<_Scalar, _RowsA> vecteur(a.rows());

        for (int i = 0; i < a.rows(); i++) {
            vecteur(i) = a(i) + b(i);
        }

        return vecteur;
    }

    /**
     * Soustraction : Vecteur - Vecteur
     */
    template <typename _Scalar, int _RowsA, int _RowsB>
    Vector<_Scalar, _RowsA> operator-(const Vector<_Scalar, _RowsA>& a, const Vector<_Scalar, _RowsB>& b)
    {
        assert(a.size() == b.size());
        Vector<_Scalar, _RowsA> vecteur(a.rows());

        for (int i = 0; i < a.rows(); i++) {
            vecteur(i) = a(i) - b(i);
        }

        return vecteur;
    }
}
