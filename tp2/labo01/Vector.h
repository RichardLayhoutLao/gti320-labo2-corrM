#pragma once

/**
 * @file Vector.h
 *
 * @brief Implémentation de vecteurs simples
 *
 * Nom: Richard Layhout Lao
 * Code permanent : LAO19089501
 * Email : richard-layhout.lao.1@ens.etsmtl.ca
 *
 */

#include <cmath>
#include "MatrixBase.h"

namespace gti320 {

    /**
     * Classe vecteur générique.
     *
     * Cette classe réutilise la classe `MatrixBase` et ses spécialisations de
     * templates pour les manipulation bas niveau. 
     */
    template <typename _Scalar = double, int _Rows = Dynamic>
      class Vector : public MatrixBase<_Scalar, _Rows, 1> {
      public:

        /**
         * Constructeur par défaut
         */
        Vector() : MatrixBase<_Scalar, _Rows, 1>() { }

        /**
         * Contructeur à partir d'un taille (rows).
         */
        explicit Vector(int rows) : MatrixBase<_Scalar, _Rows, 1>(rows, 1) { }

        /**
         * Constructeur de copie
         */
        Vector(const Vector& other) : MatrixBase<_Scalar, _Rows, 1>(other) { }

        /**
         * Destructeur
         */
        ~Vector() { }

        /**
         * Opérateur de copie
         */
        Vector& operator=(const Vector& other)
          {
            if (this != &other){
                this->m_storage = other.m_storage;
            }
           
            return *this;
          }

        /**
         * Accesseur à une entrée du vecteur (lecture seule)
         */
        _Scalar operator()(int i) const
          {
            return (double) Vector::m_storage.data()[i];
          }

        /**
         * Accesseur à une entrée du vecteur (lecture et écriture)
         */
        _Scalar& operator()(int i)                                                  
          {
            _Scalar& referenceTableauEntry = Vector::m_storage.data()[i];
            return referenceTableauEntry;
          }

        /**
         * Modifie le nombre de lignes du vecteur
         */
        void resize(int _rows)
          {
            MatrixBase<_Scalar, _Rows, 1>::resize(_rows, 1);
          }

        /**
         * Produit scalaire de *this et other.
         */
        inline _Scalar dot(const Vector& other) const
          {
            
            double total = 0;
            for (int i = 0; i < Vector::size(); i++){
                total += Vector::m_storage.data()[i] * other.data()[i];
            }
            
            return total;
          }

        /**
         * Retourne la norme euclidienne du vecteur
         */
        inline _Scalar norm() const
          {
            double total = 0;
            for (int i = 0; i < Vector::size(); i++){

                // NOUVEAU
                total += Vector::m_storage.data()[i]* Vector::m_storage.data()[i];
            }
         
            return sqrt(total);
          }

      };

}
