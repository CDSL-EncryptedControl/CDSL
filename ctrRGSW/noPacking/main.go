package main

import (
	"fmt"
	"time"

	utils "github.com/CDSL-EncryptedControl/CDSL/utils"
	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
)

func main() {
	// *****************************************************************
	// ************************* User's choice *************************
	// *****************************************************************
	// ============== Encryption parameters ==============
	// Refer to ``Homomorphic encryption standard''
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		// log2 of polynomial degree
		LogN: 12,
		// Size of ciphertext modulus (Q)
		LogQ: []int{56},
		// Size of special modulus (P)x
		LogP:    []int{51},
		NTTFlag: true,
	})
	fmt.Println("Degree of polynomials:", params.N())
	fmt.Println("Ciphertext modulus:", params.QBigInt())
	fmt.Println("Special modulus:", params.PBigInt())
	fmt.Println("primitive root:", params.RingQ().SubRings[0].NTTTable.NthRoot)
	fmt.Println("primitive root:", params.RingQ().SubRings[0].NTTTable.PrimitiveRoot)
	fmt.Println("primitive root:", params.RingQ().SubRings[0].N)

	// Default secret key distribution
	// Each coefficient in the polynomial is uniformly sampled in [-1, 0, 1]
	fmt.Println("Secret key distribution:", params.Xs())
	// Default error distribution
	// Each coefficient in the polynomial is sampled according to a
	// discrete Gaussian distribution with standard deviation 3.2 and bound 19.2
	fmt.Println("Error distribution:", params.Xe())

	// ============== Plant model ==============
	A := [][]float64{
		{0.9984, 0, 0.0042, 0},
		{0, 0.9989, 0, -0.0033},
		{0, 0, 0.9958, 0},
		{0, 0, 0, 0.9967},
	}
	B := [][]float64{
		{0.0083, 0},
		{0, 0.0063},
		{0, 0.0048},
		{0.0031, 0},
	}
	C := [][]float64{
		{0.5, 0, 0, 0},
		{0, 0.5, 0, 0},
	}
	// Plant initial state
	xp0 := []float64{
		1,
		1,
		1,
		1,
	}

	// ============== Pre-designed controller ==============
	// F must be an integer matrix
	F := [][]float64{
		{-1, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 2, 0},
		{0, 0, 0, 1},
	}
	G := [][]float64{
		{0.7160, -0.3828},
		{-0.8131, -1.4790},
		{0.6646, 1.1860},
		{0.0181, -0.0060},
	}
	R := [][]float64{
		{-1.7396, 0.3476},
		{0.2588, 1.3226},
		{0.5115, 2.4668},
		{0.0122, 0.0030},
	}
	H := [][]float64{
		{-0.8829, 0.0445, -0.0533, -0.0855},
		{0.1791, 0.2180, -0.2738, 0.0180},
	}
	// Controller initial state
	x0 := []float64{
		-1,
		0,
		-1,
		0,
	}

	// ============== Quantization parameters ==============
	s := 1 / 10000.0
	L := 1 / 10000.0
	r := 1 / 1000.0
	fmt.Printf("Scaling parameters 1/L: %v, 1/s: %v, 1/r: %v \n", 1/L, 1/s, 1/r)
	// *****************************************************************
	// *****************************************************************

	// ============== Encryption settings ==============
	// Set parameters
	levelQ := params.QCount() - 1
	levelP := params.PCount() - 1
	ringQ := params.RingQ()

	// Generate keys
	kgen := rlwe.NewKeyGenerator(params)
	sk := kgen.GenSecretKeyNew()
	rlk := kgen.GenRelinearizationKeyNew(sk)
	evk := rlwe.NewMemEvaluationKeySet(rlk)

	// Define encryptor and evaluator
	encryptorRLWE := rlwe.NewEncryptor(params, sk)
	decryptorRLWE := rlwe.NewDecryptor(params, sk)
	encryptorRGSW := rgsw.NewEncryptor(params, sk)
	evaluator := rgsw.NewEvaluator(params, evk)

	// ==============  Encryption of controller ==============
	// Quantization
	GBar := utils.ScalMatMult(1/s, G)
	RBar := utils.ScalMatMult(1/s, R)
	HBar := utils.ScalMatMult(1/s, H)

	// Encryption
	ctF := RGSW.Enc(F, encryptorRGSW, levelQ, levelP, params)
	ctG := RGSW.Enc(GBar, encryptorRGSW, levelQ, levelP, params)
	ctH := RGSW.Enc(HBar, encryptorRGSW, levelQ, levelP, params)
	ctR := RGSW.Enc(RBar, encryptorRGSW, levelQ, levelP, params)

	// ============== Simulation ==============
	// Number of simulation steps
	iter := 1000
	fmt.Printf("Number of iterations: %v\n", iter)

	// *****************
	// 1) Plant + unencrypted (original) controller
	// *****************

	// Data storage
	yUnenc := [][]float64{}
	uUnenc := [][]float64{}
	xcUnenc := [][]float64{}
	xpUnenc := [][]float64{}

	xpUnenc = append(xpUnenc, xp0)
	xcUnenc = append(xcUnenc, x0)

	// Plant state
	xp := xp0
	// Controller state
	x := x0

	for i := 0; i < iter; i++ {
		y := utils.MatVecMult(C, xp)
		u := utils.MatVecMult(H, x)
		xp = utils.VecAdd(utils.MatVecMult(A, xp), utils.MatVecMult(B, u))
		x = utils.VecAdd(utils.MatVecMult(F, x), utils.MatVecMult(G, y))
		x = utils.VecAdd(x, utils.MatVecMult(R, u))

		yUnenc = append(yUnenc, y)
		uUnenc = append(uUnenc, u)
		xcUnenc = append(xcUnenc, x)
		xpUnenc = append(xpUnenc, xp)
	}

	// *****************
	// 2) Plant + encrypted controller
	// *****************

	// State and output storage
	yEnc := [][]float64{}
	uEnc := [][]float64{}
	xpEnc := [][]float64{}
	xpEnc = append(xpEnc, xp0)

	// Plant state
	xp = xp0

	// Controller state encryption
	// xBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), x0))

	xBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), x0))
	xCt := RLWE.Enc(xBar, 1/L, *encryptorRLWE, ringQ, params)

	// For time check
	period := make([][]float64, iter)
	startPeriod := make([]time.Time, iter)

	for i := 0; i < iter; i++ {
		// **** Sensor ****
		// Plant output
		y := utils.MatVecMult(C, xp)

		startPeriod[i] = time.Now()

		// Quantize and encrypt plant output
		yBar := utils.RoundVec(utils.ScalVecMult(1/r, y))
		yCt := RLWE.Enc(yBar, 1/L, *encryptorRLWE, ringQ, params)

		// **** Encrypted Controller ****
		// Compute output
		uCt := RGSW.Mult(xCt, ctH, evaluator, ringQ, params)

		// **** Actuator ****
		// Decrypt output
		u := RLWE.Dec(uCt, *decryptorRLWE, r*s*s*L, ringQ, params)

		// Re-encrypt output
		uBar := utils.RoundVec(utils.ScalVecMult(1/r, u))
		uReEnc := RLWE.Enc(uBar, 1/L, *encryptorRLWE, ringQ, params)

		// **** Encrypted Controller ****
		// State update
		FxCt := RGSW.Mult(xCt, ctF, evaluator, ringQ, params)
		GyCt := RGSW.Mult(yCt, ctG, evaluator, ringQ, params)
		RuCt := RGSW.Mult(uReEnc, ctR, evaluator, ringQ, params)
		xCt = RLWE.AddVec(FxCt, GyCt, RuCt, params)

		period[i] = []float64{float64(time.Since(startPeriod[i]).Nanoseconds()) / 1000}

		// **** Plant ****
		// State update
		xp = utils.VecAdd(utils.MatVecMult(A, xp), utils.MatVecMult(B, u))

		// Save data
		yEnc = append(yEnc, y)
		uEnc = append(uEnc, u)
		xpEnc = append(xpEnc, xp)
	}

	avgPeriod := utils.Average(utils.MatToVec(period))
	fmt.Println("Average elapsed time for a control period:", avgPeriod, "ms")

	// Compare plant input between 1) and 2)
	uDiff := make([][]float64, iter)
	for i := range uDiff {
		uDiff[i] = []float64{utils.Vec2Norm(utils.VecSub(uUnenc[i], uEnc[i]))}
	}

	// =========== Export data ===========

	// Plant state equipped with encrypted controller
	utils.DataExport(xpEnc, "./state.csv")

	// Plant intput from encrypted controller
	utils.DataExport(uEnc, "./uEnc.csv")

	// Plant output with encrypted controller
	utils.DataExport(yEnc, "./yEnc.csv")

	// Performance of encrypted controller
	utils.DataExport(uDiff, "./uDiff.csv")

	// Elapsed time
	utils.DataExport(period, "./period.csv")

}
