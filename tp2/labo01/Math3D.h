#pragma once

/**
 * @file Math3D.h
 *
 * @brief Fonctions pour l'intinialisation et la manipulation de matrices de
 * rotation, des matrices de transformations en coordonnées homogènes et les
 * vecteurs 3D.
 * 
 * Nom: Richard Layhout Lao
 * Code permanent : LAO19089501
 * Email : richard-layhout.lao.1@ens.etsmtl.ca
 *
 */

#include "Matrix.h"
#include "Vector.h"
#include "Operators.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <math.h>


namespace gti320 {

    // Deux types de vecteurs 3D considérés ici
    typedef Vector<double, 3> Vector3d;
    typedef Vector<float, 3> Vector3f;

    // Dans le cadre de ce projet, nous considérons seulement deux
    // cas :
    //
    //  - les rotations
    //  - les translations
    //
    // Deux types de matrices en coordonnées homogèes :
    typedef Matrix<double, 4, 4, ColumnStorage> Matrix4d;
    typedef Matrix<float, 4, 4, ColumnStorage> Matrix4f;
    // 
    // Deux types de matrices pour les rotations
    typedef Matrix<double, 3, 3, ColumnStorage> Matrix3d;
    typedef Matrix<float, 3, 3, ColumnStorage> Matrix3f;


    /**
     * Calcul de la matrice inverse, SPÉCIALISÉ pour le cas d'une matrice de
     * transformation en coordonnées homogènes.
     */
    template<> inline
      Matrix4d Matrix4d::inverse() const
        {
            // Note :  Au départ, je l'ai implémenté de cette facon afin de trouver la matrice inverse.
            //         Puis, j'ai réalisé vers la fin que ce n'est pas la meilleure méthode pour trouver 
            //         la matrice inverse spécialisé pour le cas d'une matrice de transformation en coordonnées homogènes.
            //         Par manque de temps et de déboggage, j'ai décicé d'implémenter de garder l'implémentation initial.
            // 
            //         Je me suis inspiré par les liens suivants:   http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche0023.html
            //                                                      https://stackoverflow.com/questions/23445695/efficiently-solve-ax-b-where-a-is-a-4x4-symmetric-metrix-and-b-is-4x1-vector
            //          


            Matrix<double, 4, 4, ColumnStorage> matriceInverse;

            double a11 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[0];
            double a21 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[1];
            double a31 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[2];
            double a41 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[3];

            double a12 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[4];
            double a22 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[5];
            double a32 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[6];
            double a42 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[7];

            double a13 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[8];
            double a23 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[9];
            double a33 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[10];
            double a43 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[11];

            double a14 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[12];
            double a24 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[13];
            double a34 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[14];
            double a44 = Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[15];
            
            double determinant
                = (a11 * a22 * a33 * a44) + (a11 * a23 * a34 * a42) + (a11 * a24 * a32 * a43)
                + (a12 * a21 * a34 * a43) + (a12 * a23 * a31 * a44) + (a12 * a24 * a33 * a41)
                + (a13 * a21 * a32 * a44) + (a13 * a22 * a34 * a41) + (a13 * a24 * a31 * a42)
                + (a14 * a21 * a33 * a42) + (a14 * a22 * a31 * a43) + (a14 * a23 * a32 * a41)
                - (a11 * a22 * a34 * a43) - (a11 * a23 * a32 * a44) - (a11 * a24 * a33 * a42)
                - (a12 * a21 * a33 * a44) - (a12 * a23 * a34 * a41) - (a12 * a24 * a31 * a43)
                - (a13 * a21 * a34 * a42) - (a13 * a22 * a31 * a44) - (a13 * a24 * a32 * a41)
                - (a14 * a21 * a32 * a43) - (a14 * a22 * a33 * a41) - (a14 * a23 * a31 * a42);


            matriceInverse(0, 0) = (a22 * a33 * a44 + a23 * a34 * a42 + a24 * a32 * a43 - a22 * a34 * a43 - a23 * a32 * a44 - a24 * a33 * a42) / determinant;
            matriceInverse(1, 0) = (a21 * a34 * a43 + a23 * a31 * a44 + a24 * a33 * a41 - a21 * a33 * a44 - a23 * a34 * a41 - a24 * a31 * a43) / determinant;
            matriceInverse(2, 0) = (a21 * a32 * a44 + a22 * a34 * a41 + a24 * a31 * a42 - a21 * a34 * a42 - a22 * a31 * a44 - a24 * a32 * a41) / determinant;
            matriceInverse(3, 0) = (a21 * a33 * a42 + a22 * a31 * a43 + a23 * a32 * a41 - a21 * a32 * a43 - a22 * a33 * a41 - a23 * a31 * a42) / determinant;
            matriceInverse(0, 1) = (a12 * a34 * a43 + a13 * a32 * a44 + a14 * a33 * a42 - a12 * a33 * a44 - a13 * a34 * a42 - a14 * a32 * a43) / determinant;
            matriceInverse(1, 1) = (a11 * a33 * a44 + a13 * a34 * a41 + a14 * a31 * a43 - a11 * a34 * a43 - a13 * a31 * a44 - a14 * a33 * a41) / determinant;
            matriceInverse(2, 1) = (a11 * a34 * a42 + a12 * a31 * a44 + a14 * a32 * a41 - a11 * a32 * a44 - a12 * a34 * a41 - a14 * a31 * a42) / determinant;
            matriceInverse(3, 1) = (a11 * a32 * a43 + a12 * a33 * a41 + a13 * a31 * a42 - a11 * a33 * a42 - a12 * a31 * a43 - a13 * a32 * a41) / determinant;
            matriceInverse(0, 2) = (a12 * a23 * a44 + a13 * a24 * a42 + a14 * a22 * a43 - a12 * a24 * a43 - a13 * a22 * a44 - a14 * a23 * a42) / determinant;
            matriceInverse(1, 2) = (a11 * a24 * a43 + a13 * a21 * a44 + a14 * a23 * a41 - a11 * a23 * a44 - a13 * a24 * a41 - a14 * a21 * a43) / determinant;
            matriceInverse(2, 2) = (a11 * a22 * a44 + a12 * a24 * a41 + a14 * a21 * a42 - a11 * a24 * a42 - a12 * a21 * a44 - a14 * a22 * a41) / determinant;
            matriceInverse(3, 2) = (a11 * a23 * a42 + a12 * a21 * a43 + a13 * a22 * a41 - a11 * a22 * a43 - a12 * a23 * a41 - a13 * a21 * a42) / determinant;
            matriceInverse(0, 3) = (a12 * a24 * a33 + a13 * a22 * a34 + a14 * a23 * a32 - a12 * a23 * a34 - a13 * a24 * a32 - a14 * a22 * a33) / determinant;
            matriceInverse(1, 3) = (a11 * a23 * a34 + a13 * a24 * a31 + a14 * a21 * a33 - a11 * a24 * a33 - a13 * a21 * a34 - a14 * a23 * a31) / determinant;
            matriceInverse(2, 3) = (a11 * a24 * a32 + a12 * a21 * a34 + a14 * a22 * a31 - a11 * a22 * a34 - a12 * a24 * a31 - a14 * a21 * a32) / determinant;
            matriceInverse(3, 3) = (a11 * a22 * a33 + a12 * a23 * a31 + a13 * a21 * a32 - a11 * a23 * a32 - a12 * a21 * a33 - a13 * a22 * a31) / determinant;
                        
            return matriceInverse;
        }

    /**
     * Calcul de la matrice inverse, SPÉCIALISÉ pour le cas d'une matrice de
     * rotation.
     *
     * (vous pouvez supposer qu'il s'agit d'une matrice de rotation)
     */
    template<> inline
      Matrix3d Matrix3d::inverse() const
        {
            Matrix<double, 3, 3, ColumnStorage> matriceInverse;
            int total = 0;
            for (int i = 0; i < Matrix<double, 3, 3, ColumnStorage>::rows(); i++) {
                for (int j = 0; j < Matrix<double, 3, 3, ColumnStorage>::cols(); j++) {
                    matriceInverse(i, j) = Matrix<double, 3, 3, ColumnStorage>::m_storage.data()[total];
                    total++;
                }
            }

            return matriceInverse;
        }


    /**
     * Multiplication d'une matrice 4x4 avec un vecteur 3D où la matrice
     * représente une transformation en coordonnées homogène.
     */ 
    template <typename _Scalar>
      Vector<_Scalar, 3> operator*(const Matrix<_Scalar, 4, 4, ColumnStorage>& A, const Vector<_Scalar, 3>& v)
        {
          double t[3];
          double total;
          Vector<_Scalar, 3> vecteur(v.rows());
          
          for (int i = 0; i < 3; i++) {
              t[i] = A(i, 3);
          }

          for (int i = 0; i < A.rows()-1; i++)     {
              total = 0;
              for (int j = 0; j < A.cols()-1; j++)
              {
                  
                  total += A(i, j) * v(j) ;
              }
              total = total + t[i];
              vecteur(i) = total;

          }

          return vecteur;
        }


    /**
     * Initialise et retourne la matrice de rotation définie par les angles
     * d'Euler XYZ exprimés en radians.
     *
     * La matrice doit correspondre au produit : Rz*Ry*Rx.

     */
    template<typename _Scalar>
      static Matrix<_Scalar, 3, 3> makeRotation(_Scalar x, _Scalar y, _Scalar z)
        {
          double cosValueX = cos(x);
          double sinValueX = sin(x);
          double cosValueY = cos(y);
          double sinValueY = sin(y);
          double cosValueZ = cos(z);
          double sinValueZ = sin(z);


          Matrix<_Scalar, 3, 3> rxyz(3, 3);

          rxyz(0, 0) = cosValueY * cosValueZ;
          rxyz(0, 1) = sinValueX * sinValueY * cosValueZ - cosValueX * sinValueZ;
          rxyz(0, 2) = cosValueX * sinValueY * cosValueZ + sinValueX * sinValueZ;
          rxyz(1, 0) = cosValueY * sinValueZ;
          rxyz(1, 1) = sinValueX * sinValueY * sinValueZ + cosValueX * cosValueZ;
          rxyz(1, 2) = cosValueX * sinValueY * sinValueZ - sinValueX * cosValueZ;
          rxyz(2, 0) = -sinValueY;
          rxyz(2, 1) = sinValueX * cosValueY;
          rxyz(2, 2) = cosValueX * cosValueY;

          return rxyz;
        }

    /**
     * Initialise et retourne la matrice identité
     */
    template<>
      void Matrix4d::setIdentity()
        {
          Matrix<double, 4, 4, ColumnStorage>::m_storage.setZero();
          for (int i = 0; i < Matrix<double, 4, 4, ColumnStorage>::m_storage.size(); i += Matrix<double, 4, 4, ColumnStorage>::cols()) {
              Matrix<double, 4, 4, ColumnStorage>::m_storage.data()[i] = 1;
              i++;
          }
        }

}
