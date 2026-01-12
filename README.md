# CDSL: Cryptography for Dynamic Systems Library


CDSL provides codes for implementing secure dynamic systems based on modern cryptography.
The library features linear dynamic controllers operating over homomorphically encrypted data implemented using [Lattigo](https://github.com/tuneinsight/lattigo) version 6.1.0.
The encrypted controllers are designed based on the state-of-the-art methods developed by CDSL, [SNU](https://post.cdsl.kr/) and [SEOULTECH](https://junsookim4.wordpress.com/).


---

### Features


This code provides two methods to operate the linear dynamic controller over encrypted data, using a Ring-LWE based cryptosystem. 


- `ctrRGSW` [1]: Supports unlimited number of recursive homomorhpic multiplications without the use of bootstrapping. More specifically, the encrypted controller state is recursively multiplied to the encrypted state matrix without decryption. The effect of error growth is suppressed by the stability of the closed-loop system. 
    - `ctrRGSW/noPacking`: Naive implementation that does not use packing. 
    - `ctrRGSW/packing`: A novel "coefficient packing" technique is applied, resulting in enhanced computation speed and memory efficiency   
    




- `ctrRLWE`: This implementation is based on the encrypted controller design proposed in Section IV of [2].
    The encrypted controller is non-recursive, although it is based on a dynamic controller.
    This is enabled by ``re-encryption'' of the encrypted controller output at the actuator, which requires an additional communication link between the plant and the controller.
    However, under sufficiently fast communication, the encrypted controller operates faster than that of `ctrRGSW`.
    The example code is based on the BGV scheme, but this can be easily modified to use other Ring-LWE based schemes.

- In this example, we use the four-tank system [7] as the plant and pre-design an observer-based controller. Please refer to [8] for more details regarding the setting.


- `integer_ctr` [9]: Provides algorithms for design of controllers having integer coefficients, a property that is required to implement linear dynamic controllers over encrypted data.
    - `stabilization`: Finds a stabilizing controller having integer coefficients for any given discrete-time LTI plant. (Section III of [9])

---

### How to use
1. Download or clone this repository using
```
git clone https://github.com/CDSL-EncryptedControl/CDSL.git
```


2. Change the directory to a folder you wish to use and run the `main.go` file. For example,

```
cd ctrRGSW/noPacking
go run main.go  
```
or
```
cd ctrRGSW/packing
go run main.go  
```
or
```
cd ctrRLWE
go run main.go  
```

---

### License
Distributed under the MIT License. See `LICENSE` for more information.

---

### Contributing

We **greatly appreciate** any typo and factual corrections, no matter how small the change is.

If you spot an error in our code, please report it to us without hesitation.
If the issue is not completely obvious, please provide justifications and details as necessary. 

---


### Contact

Yeongjun Jang - jangyj@cdsl.kr

Joowon Lee - jwlee@cdsl.kr

Junsoo Kim - junsookim@seoultech.ac.kr

---

### Acknowledgements
- This work was supported by the National Research Foundation of Korea(NRF) grant funded by the Korea government(MSIT) (No. RS-2024-00353032).
- Special thanks to [Seonhong Min](https://snu-lukemin.github.io/), [Hyesun Kwak](https://hyesunkwak.github.io/), and [Yongsoo Song](https://yongsoosong.github.io/) with the Department of Computer Science and Engineering, Seoul National University, for the great help.

---

### References
[1] [Y. Jang, J. Lee, S. Min, H. Kwak, J. Kim, and Y. Song, "Ring-LWE based encrypted controller with unlimited number of recursive multiplications and effect of error growth," 2024, arXiv:2406.14372.](https://arxiv.org/abs/2406.14372)

[2] [J. Lee, D. Lee, J. Kim, and H. Shim, "Encrypted dynamic control exploiting limited number of multiplications and a method using RLWE-based cryptosystem," _IEEE Trans. Syst. Man. Cybern.: Syst._, vol. 55, no. 1, pp. 158-169, 2025.](https://ieeexplore.ieee.org/abstract/document/10730788)

[3] [J. Kim, H. Shim, and K. Han, "Dynamic controller that operates over homomorphically encrypted data for infinite time horizon," _IEEE Trans. Autom. Control_, vol. 68, no. 2, pp. 660-672, 2023.](https://ieeexplore.ieee.org/abstract/document/9678042)

[4] [J. Kim, H. Shim, H. Sandberg, and K. H. Johansson, “Method for running dynamic systems over encrypted data for infinite time horizon without bootstrapping and re-encryption,” in Proc. 60th IEEE Conf. Decision Control, 2021, pp. 5614–5619.](https://ieeexplore.ieee.org/abstract/document/9682828?casa_token=LHR79rToQ7oAAAAA:Wz1AzFWR7VW6DYKUhLFYcoXtpMx4AIT9E_krpOpFy7QUO5lSkvPf_0ZZgPsdp65ZzaGx-ejlPA)

[5] [M. S. Tavazoei, “Non-minimality of the realizations and possessing state matrices with integer elements in linear discrete-time controllers,” IEEE Trans. Autom. Control, vol. 68, no. 6, pp. 3698–3703, 2023.](https://ieeexplore.ieee.org/abstract/document/9835020?casa_token=_rdGjQLc7ZEAAAAA:QLxzC1QlnNVYriMTL1gbSjtv5U2oTwfVO5OqVFfGS0Qpz8hx7exSuJKJ9H8XBh_qDucoZt8oBg)

[6] [J. Lee, D. Lee, S. Lee, J. Kim, and H. Shim, “Conversion of controllers to have integer state matrix for encrypted control: Non-minimal order approach,” in Proc. 62nd IEEE Conf. Decision Control, 2023, pp. 5091–5096.](https://ieeexplore.ieee.org/abstract/document/10383200?casa_token=lbob37tAZ-MAAAAA:vAVUmuIngRzHYefqaYHQM5TfukcAI7Lh1YmYngqcLYMj74Mtzq0xGybkntfWSd-DKwogxrvnxg)

[7] [K. Johansson, “The quadruple-tank process: A multivariable laboratory process with an adjustable zero,” IEEE Trans. Control Sys. Technol., vol. 8, no. 3, pp. 456–465, 2000.](https://ieeexplore.ieee.org/abstract/document/845876?casa_token=1CWEIgmKIscAAAAA:Hh3D4_xn5B8MWVoMpQHof8glwtWpGXMuddehBoKXbZAOh2WwsDlemeiWeZ6nAwQGThjhYYw1wQ)

[8] [Y. Jang, J. Lee, and J. Kim, "Documentation on encrypted dynamic control simulation code using Ring-LWE based cryptosystems," ](link)

[9] [J. Lee, D. Lee, and J. Kim, "Stabilization by controllers having integer coefficients," arXiv:2505.00481.](https://arxiv.org/abs/2505.00481)
