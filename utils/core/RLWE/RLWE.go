package RLWE

import (
	"math"

	"github.com/CDSL-EncryptedControl/CDSL/utils"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

// Unpack RLWE ciphertext
// Input
// - ctRLWE: RLWE ciphertext
// - tau   : # of total packing slots
// - n     : # of packing slots to unpack
// Output
// - ctOut: n x 1 RLWE vector
func UnpackCt(ctRLWE *rlwe.Ciphertext, n int, tau int, evaluatorRLWE *rlwe.Evaluator, ringQ *ring.Ring, monomials []ring.Poly, params rlwe.Parameters) []*rlwe.Ciphertext {
	// scale
	scalar := params.Q()[0] - uint64((params.Q()[0]+1)/uint64(tau))
	ringQ.MulScalar(ctRLWE.Value[0], scalar, ctRLWE.Value[0])
	ringQ.MulScalar(ctRLWE.Value[1], scalar, ctRLWE.Value[1])

	ctUnpack := make([]*rlwe.Ciphertext, tau)
	for i := 0; i < tau; i++ {
		ctUnpack[i] = rlwe.NewCiphertext(params, ctRLWE.Degree(), ctRLWE.Level())
	}
	ctUnpack[0] = ctRLWE
	tmpCt := rlwe.NewCiphertext(params, ctRLWE.Degree(), ctRLWE.Level())
	for i := tau; i > 1; i /= 2 {
		for j := 0; j < tau; j += i {
			// Automorphism
			evaluatorRLWE.Automorphism(ctUnpack[j], uint64(i+1), tmpCt)

			ringQ.Sub(tmpCt.Value[0], ctUnpack[j].Value[0], ctUnpack[i/2+j].Value[0])
			ringQ.Sub(tmpCt.Value[1], ctUnpack[j].Value[1], ctUnpack[i/2+j].Value[1])

			ringQ.Add(ctUnpack[j].Value[0], tmpCt.Value[0], ctUnpack[j].Value[0])
			ringQ.Add(ctUnpack[j].Value[1], tmpCt.Value[1], ctUnpack[j].Value[1])

			idx := int(math.Log2(float64(i))) - 1
			ringQ.MulCoeffsMontgomery(ctUnpack[i/2+j].Value[0], monomials[idx], ctUnpack[i/2+j].Value[0])
			ringQ.MulCoeffsMontgomery(ctUnpack[i/2+j].Value[1], monomials[idx], ctUnpack[i/2+j].Value[1])
		}
	}

	// Bit reverse
	j := 0
	for i := 1; i < tau; i += 1 {
		bit := tau >> 1
		for j >= bit {
			j -= bit
			bit >>= 1
		}
		j += bit
		if i < j {
			ctUnpack[i], ctUnpack[j] = ctUnpack[j], ctUnpack[i]
		}
	}

	// Takes the first n ciphertexts
	ctOut := make([]*rlwe.Ciphertext, n)
	for j := 0; j < n; j += 1 {
		ctOut[j] = ctUnpack[j].CopyNew()
	}

	return ctOut
}

// Encrypts float vector into an RLWE ciphertext
// Input
// - v: n x 1 float vector
// Output
// - ctOut: n x l RLWE vector
// func Enc(v []float64, scale float64, encryptorRLWE rlwe.Encryptor, ringQ *ring.Ring, params rlwe.Parameters) []*rlwe.Ciphertext {
// 	var err error

// 	row := len(v)
// 	scaleV := utils.ScalVecMult(scale, v)
// 	modV := utils.ModVecFloat(scaleV, params.Q()[0])

// 	ctOut := make([]*rlwe.Ciphertext, row)
// 	for r := 0; r < row; r++ {
// 		pt := rlwe.NewPlaintext(params, params.MaxLevel())
// 		pt.Value.Coeffs[0][0] = modV[r]
// 		ringQ.NTT(pt.Value, pt.Value)
// 		ctOut[r], err = encryptorRLWE.EncryptNew(pt)
// 		if err != nil {
// 			panic(err)
// 		}
// 	}

//		return ctOut
//	}
func Enc(v []int64, scale float64, encryptorRLWE rlwe.Encryptor, ringQ *ring.Ring, params rlwe.Parameters) []*rlwe.Ciphertext {
	var err error
	row := len(v)
	scaleV := utils.ScalVecMultInt(int64(scale), v)
	modV := utils.ModVec(scaleV, params.Q()[0])

	ctOut := make([]*rlwe.Ciphertext, row)
	for r := 0; r < row; r++ {
		pt := rlwe.NewPlaintext(params, params.MaxLevel())
		pt.Value.Coeffs[0][0] = modV[r]
		ringQ.NTT(pt.Value, pt.Value)
		ctOut[r], err = encryptorRLWE.EncryptNew(pt)
		if err != nil {
			panic(err)
		}
	}

	return ctOut
}

// Encrypts float vector into an RLWE ciphertext by PACKING
// Input
// - v: n x 1 float vector
// Output
// - ctOut: RLWE ciphertext
// func EncPack(v []float64, tau int, scale float64, encryptorRLWE rlwe.Encryptor, ringQ *ring.Ring, params rlwe.Parameters) *rlwe.Ciphertext {
// 	var err error

// 	row := len(v)
// 	scaleV := utils.ScalVecMult(scale, v)
// 	modV := utils.ModVecFloat(scaleV, params.Q()[0])

// 	ctOut := rlwe.NewCiphertext(params, 1, params.MaxLevel())
// 	pt := rlwe.NewPlaintext(params, params.MaxLevel())
// 	for r := 0; r < row; r++ {
// 		// Store in the packing slots
// 		pt.Value.Coeffs[0][params.N()*r/tau] = modV[r]
// 	}
// 	ringQ.NTT(pt.Value, pt.Value)
// 	ctOut, err = encryptorRLWE.EncryptNew(pt)
// 	if err != nil {
// 		panic(err)
// 	}

// 	return ctOut
// }

func EncPack(v []int64, tau int, scale float64, encryptorRLWE rlwe.Encryptor, ringQ *ring.Ring, params rlwe.Parameters) *rlwe.Ciphertext {
	var err error

	row := len(v)
	scaleV := utils.ScalVecMultInt(int64(scale), v)
	modV := utils.ModVec(scaleV, params.Q()[0])

	ctOut := rlwe.NewCiphertext(params, 1, params.MaxLevel())
	pt := rlwe.NewPlaintext(params, params.MaxLevel())
	for r := 0; r < row; r++ {
		// Store in the packing slots
		pt.Value.Coeffs[0][params.N()*r/tau] = modV[r]
	}
	ringQ.NTT(pt.Value, pt.Value)
	ctOut, err = encryptorRLWE.EncryptNew(pt)
	if err != nil {
		panic(err)
	}

	return ctOut
}

// 1) Decrypt RLWE vector -> integer vector
// 2) Map constant terms of integer vector [0,q/2) -> [-q/2, q/2)
// 3) Scale down
// Input
// - ctRLWE: n x 1 RLWE vector
// Output
// - valOut: n x 1 float vector
func Dec(ctRLWE []*rlwe.Ciphertext, decryptorRLWE rlwe.Decryptor, scale float64, ringQ *ring.Ring, params rlwe.Parameters) []float64 {
	row := len(ctRLWE)
	q := float64(params.Q()[0])
	offset := uint64(q / (scale * 2.0))
	valOut := make([]float64, row)
	for r := 0; r < row; r++ {
		ringQ.AddScalar(ctRLWE[r].Value[0], offset, ctRLWE[r].Value[0])
		pt := decryptorRLWE.DecryptNew(ctRLWE[r])
		if pt.IsNTT {
			params.RingQ().INTT(pt.Value, pt.Value)
		}
		ringQ.SubScalar(ctRLWE[r].Value[0], offset, ctRLWE[r].Value[0])
		// ringQ.SubScalar(pt.Value, offset, pt.Value)
		// Constant terms
		val := float64(pt.Value.Coeffs[0][0])
		// Mapping to [-q/2, q/2)
		val = val - math.Floor((val+q/2.0)/q)*q
		// Scale down
		valOut[r] = val * scale
	}
	return valOut
}

// 1) Decrypt RLWE ciphertext
// 2) Unpack first m slots -> integer vector
// 3) rescale
// Input
// - ctRLWE: RLWE
// Output
// - valOut: m x 1 float vector
func DecUnpack(ctRLWE *rlwe.Ciphertext, m int, tau int, decryptorRLWE rlwe.Decryptor, scale float64, ringQ *ring.Ring, params rlwe.Parameters) []float64 {
	q := float64(params.Q()[0])
	offset := uint64(q / (scale * 2.0))
	valOut := make([]float64, m)

	ringQ.AddScalar(ctRLWE.Value[0], offset, ctRLWE.Value[0])
	pt := decryptorRLWE.DecryptNew(ctRLWE)
	if pt.IsNTT {
		params.RingQ().INTT(pt.Value, pt.Value)
	}
	ringQ.SubScalar(ctRLWE.Value[0], offset, ctRLWE.Value[0])
	// ringQ.SubScalar(pt.Value, offset, pt.Value)
	// Constant terms
	for r := 0; r < m; r++ {
		val := float64(pt.Value.Coeffs[0][params.N()*r/tau])
		// Mapping to [-q/2, q/2)
		val = val - math.Floor((val+q/2.0)/q)*q
		// Scale down
		valOut[r] = val * scale
	}
	return valOut
}

// Add RLWE vectors
// Input
// - ctRLWE1: n x 1 RLWE vector
// - ctRLWE2: n x 1 RLWE vector
// - ctRLWE3: n x 1 RLWE vector
// Output
// - ctOut: n x 1 RLWE vector
func AddVec(ctRLWE1 []*rlwe.Ciphertext, ctRLWE2 []*rlwe.Ciphertext, ctRLWE3 []*rlwe.Ciphertext, params rlwe.Parameters) []*rlwe.Ciphertext {
	row := len(ctRLWE1)
	ctOut := make([]*rlwe.Ciphertext, row)
	for r := 0; r < row; r++ {
		ctOut[r] = rlwe.NewCiphertext(params, ctRLWE2[0].Degree(), ctRLWE2[0].Level())
		params.RingQ().Add(ctRLWE1[r].Value[0], ctRLWE2[r].Value[0], ctOut[r].Value[0])
		params.RingQ().Add(ctOut[r].Value[0], ctRLWE3[r].Value[0], ctOut[r].Value[0])
		params.RingQ().Add(ctRLWE1[r].Value[1], ctRLWE2[r].Value[1], ctOut[r].Value[1])
		params.RingQ().Add(ctOut[r].Value[1], ctRLWE3[r].Value[1], ctOut[r].Value[1])
	}

	return ctOut
}

// Add RLWE ciphertexts
// Input
// - ctRLWE1: RLWE ciphertext
// - ctRLWE2: RLWE ciphertext
// Output
// - ctOut: RLWE ciphertext
func Add(ctRLWE1 *rlwe.Ciphertext, ctRLWE2 *rlwe.Ciphertext, ctRLWE3 *rlwe.Ciphertext, params rlwe.Parameters) *rlwe.Ciphertext {
	ctOut := rlwe.NewCiphertext(params, ctRLWE2.Degree(), ctRLWE2.Level())

	params.RingQ().Add(ctRLWE1.Value[0], ctRLWE2.Value[0], ctOut.Value[0])
	params.RingQ().Add(ctOut.Value[0], ctRLWE3.Value[0], ctOut.Value[0])
	params.RingQ().Add(ctRLWE1.Value[1], ctRLWE2.Value[1], ctOut.Value[1])
	params.RingQ().Add(ctOut.Value[1], ctRLWE3.Value[1], ctOut.Value[1])

	return ctOut
}
