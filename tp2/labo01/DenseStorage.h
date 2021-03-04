#pragma once

/**
 * @file DenseStorage.h
 *
 * @brief Stockage dense pour des données à taille fixe ou dynamique.
 *
 * Nom: Richard Layhout Lao
 * Code permanent : LAO19089501
 * Email : richard-layhout.lao.1@ens.etsmtl.ca
 *
 */

#include <cstring>
#include <cassert>

namespace gti320
{

  enum SizeType
    {
      Dynamic = -1
    };

  /**
   * Stockage à taille fixe.
   *
   * Le nombre de données à stocker est connu au moment de la compilation.
   * Ce nombre est donné par le paramètre de patron : _Size
   *
   * Un tampon (tableau) de taille `_Size_` est alloué sur la pile
   * d'exécution.
   *
   */
  template<typename _Scalar, int _Size> class DenseStorage
    {
  private:
    //Déclarer une variable m_data et allouer la mémoire pour y stocker _Size éléments
    _Scalar m_data[_Size];         

  public:

    /**
     * Constructeur par défaut
     */
    DenseStorage() { }

    /**
     * Constructeur de copie
     */
    DenseStorage(const DenseStorage& other)
      {
        memcpy(m_data, other.m_data, sizeof(m_data));
      }

    /**
     * Constructeur avec taille spécifiée
     *
     * (doit être la même que la taille spécifiée dans le patron)
     */
    explicit DenseStorage(int _size) { }

    /**
     * Constructor avec taille (_size) et données initiales (_data).
     */
    explicit DenseStorage(const _Scalar* _data, int _size)
      {
        memcpy(m_data, _data, sizeof(_Scalar) * _size);
      }

    /**
     * Opérateur de copie
     */
    DenseStorage& operator=(const DenseStorage& other)
      {

        if (this != &other) {
            memcpy(m_data, other.m_data, sizeof(m_data));
        }

            return *this;
      }

    static int size() { return _Size; }

    /**
     * Redimensionne le stockage pour qu'il contienne `size` élément.
     */
    void resize(int size)
      {
        // Do nothing, not valid for fixed matrices.
      }

    /**
     * Mets tous les éléments à zéro.
     */
    void setZero()
      {
        memset(m_data, 0, sizeof(_Scalar) * _Size);
      }

    /**
     * Accès au tampon de données (en lecteur seulement)
     */
    const _Scalar* data() const
      {
        return m_data;
      }

    /**
     * Accès au tampon de données (pour lecture et écriture)
     */
    _Scalar* data()
      {
        return m_data;
      }
    };



  /**
   * Stockage à taille dynamique.
   *
   * Le nombre de données à stocker est déterminé à l'exécution.
   * Un tampon de la taille demandée doit être alloué sur le tas via
   * l'opérateur `new []` et la mémoire doit être libérée avec `delete[]`
   */
  template<typename _Scalar>
    class DenseStorage<_Scalar, Dynamic>
      {
    private:
      _Scalar* m_data;
      int m_size;

    public:

      /**
       * Constructeur par défaut
       */
      DenseStorage() : m_data(nullptr), m_size(0) {}

      /**
       * Constructeur avec taille spécifiée
       */
      explicit DenseStorage(int _size) : m_data(nullptr), m_size(_size)
        {
          //Allouer un tampon pour stocker _size éléments de type _Scalar.
          m_data = new _Scalar[_size];
          this->m_size = _size;

          //Initialiser ce tampon à zéro.                                                     
          setZero();
        }

      /**
       * Constructeur de copie
       */
      DenseStorage(const DenseStorage& other)
        : m_data(nullptr)
          , m_size(other.m_size)
        {
          //Allouer un tampon pour stocker _size éléments de type _Scalar.                       
          m_data = new _Scalar[m_size];
          memcpy(m_data, other.m_data, sizeof(_Scalar) * m_size);
        }

      /**
       * Opérateur de copie
       */
      DenseStorage& operator=(const DenseStorage& other)
        {

          if (this != &other) {
              
              if (m_size != other.size())
              {
              resize(other.size());

              }
              memcpy(m_data, other.m_data, sizeof(_Scalar) * other.m_size);
          }


          return *this;
        }

      /**
       * Destructeur
       */
      ~DenseStorage()
        {
            delete[] m_data;
        }

      /**
       * Retourne la taille du tampon 
       */
      inline int size() const { return m_size; }

      /**
       * Redimensionne le tampon alloué pour le stockage.
       * La mémoire qui n'est plus utilisée doit être libérée.
       */
      void resize(int _size)
        {


          if (m_size != _size) {
            m_size = _size;
            delete[] m_data;
            m_data = new _Scalar[m_size];
          }


        }

      /**
       * Met tous les éléments à zéro.
       */
      void setZero()
        {                                   
          memset(m_data, 0, sizeof(_Scalar) * m_size);
        }

      /**
       * Accès au tampon de données (en lecteur seulement)
       */
      const _Scalar* data() const { return m_data; }

      /**
       * Accès au tampon de données (pour lecture et écriture)
       */
      _Scalar* data() { return m_data; }
      };

}
