/**
  ******************************************************************************
  * @file    sine_model_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Wed Aug 30 15:27:13 2023
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "sine_model_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_sine_model_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_sine_model_weights_array_u64[297] = {
  0xbf121612U, 0x0U, 0xbbb7495600000000U, 0xbd31c979U,
  0x0U, 0x0U, 0xbb4cd02abf508541U, 0x3f23b96aU,
  0xbed5bd263ead262bU, 0x3e3a00163e24578aU, 0xbe9b9f0e3ead222dU, 0xbea396fc3edcb22aU,
  0xbed957c73ea06bcdU, 0xbe82c42cbbb0e2c0U, 0xbec3ecdebe940e87U, 0x3edcc63d3e10962bU,
  0xbcc13cb03e8a59e5U, 0xbe2292a23cb8dae0U, 0xbe4d3d3fbe8876b2U, 0x3eb82915be3c343dU,
  0xbe621bad3dcf4554U, 0xbe96ebb8be147362U, 0xbe0de5983e44aa86U, 0x3c4699c0bc53c6e0U,
  0x3e7f11363c9ecef0U, 0x3ed1dd833d34eb78U, 0x3ed7861dbdc91b88U, 0xbe6a5448be9ac34cU,
  0x3dea289cbeaea03eU, 0xbece6f02be9de4a4U, 0x3e944eb3be96e124U, 0x3e817223bebe1baaU,
  0xbe3366e63e331d5cU, 0xbe998ee2be6c355bU, 0x3e055d643edba479U, 0xbd45c6703eab1378U,
  0xbdd5534c3e4cce0aU, 0x3ec7e0573dd5a41cU, 0xbe87e99b3e91c4d6U, 0xbebe674ebe021896U,
  0x3da8959c3f1d5d43U, 0x3eabcdcb3e922021U, 0xbdac286fbe9477d0U, 0x3ebf387b3ecac027U,
  0x3ea6b535bebbb8d8U, 0xbd8e6de43e700e92U, 0xbe4fedf6be4c6e22U, 0x3eca8ff3bded93b2U,
  0x3e290e52bd8976e4U, 0xbe78bacabe9f5394U, 0x3ea03cbbbed46b85U, 0xbeb20194be20bfeeU,
  0x3e6055e2beaea9c9U, 0xbe7d31283ec6de33U, 0xbdb58c20be1d7048U, 0x3c916210be5702b5U,
  0xbdfb5848be826364U, 0x3eaf35cf3e8ab7abU, 0xbdaab998bbd54f40U, 0xbdcc6fdcbecc8eb0U,
  0xbe91bf42bea72738U, 0xbd5f9f18bec9f02eU, 0xbeb90febbeba88c6U, 0xbec27a6fbeb3aa80U,
  0x3ed6da233e30f84eU, 0xbe79b464bcc05bf0U, 0x3e305422bd999224U, 0xbed72979be4ec203U,
  0xbdf439743e15145aU, 0x3e916ecb3e595366U, 0x3dfb7bd4be9fcbd8U, 0x3c80f6a0bea64390U,
  0xbea3d3f23e39d29eU, 0x3c8221303eaa81edU, 0xbdc486e0be07027aU, 0x3e361c823d431c58U,
  0xbe9da14e3e5931c6U, 0xbec651c7be3aee04U, 0xbe85e330becc5a52U, 0x3b73f300bd1d2bc0U,
  0x3b7ea180bf81f2aeU, 0xbd9900b43eb366e9U, 0x3ecdffa4bc2c81a0U, 0x3ec453e73ebc8306U,
  0x3e22194a3e893453U, 0xbe2926843e8e4bd7U, 0x3ed5c42bbf685f45U, 0x3da341943ead9b53U,
  0xbe189f163f47eddcU, 0xbec5ad5b3d237dd0U, 0xbe6b2b70bec19c6fU, 0xbde58bfc3e59b244U,
  0x3e8b97833daf0d64U, 0xbed74873bea2ef3cU, 0x3d5b99cd3f0f2949U, 0x3c9693a0bed91425U,
  0x3dbac5a4be8d2fe5U, 0x3e929033bd7f9260U, 0xbda653fcbd924528U, 0xbe5cca953ecefc55U,
  0x3e9ae1713e9994b7U, 0x3edacdd13ed6be0fU, 0xbe9bc2c63e1a915aU, 0xbd276710bea0e9b4U,
  0xbed4b1ca3e5fd4dbU, 0x3c81cb70bd9f0604U, 0x3e85b860bd9291f4U, 0x3eaf68713dd9e9c7U,
  0x3e97f6ff3c5d5200U, 0x3dfd78a4bd8bdd54U, 0x3ed133b0bd04d98fU, 0x3da376fcbe48c607U,
  0xbe4c961a3d8f2a94U, 0xbec37d523ea4f027U, 0xbeb0f4aa3e7f03e2U, 0xbe8d3b12beb47883U,
  0xbce04a90bec22115U, 0x3d2c9430bd1c8098U, 0x3ce9050a3e92b5e9U, 0xbe54004c3cd92ad2U,
  0xbe71038cbf2463ecU, 0x3eba5f0fbeb3a65cU, 0xbe2015b23ed6c217U, 0xbe827edc3e15730dU,
  0xbe8d0dcc3eda738dU, 0xbde123483e043182U, 0x3e508b46beb667a0U, 0x3e5a17563d41758cU,
  0x3e87a0e9bed9e292U, 0x3e5da1d63d3c7f60U, 0xbe9205983b9fcc80U, 0xbe4c9ef4bdb7cc24U,
  0xbe8934fdbedafef0U, 0xbbe38a803e0b53beU, 0xbec284efbecc7937U, 0x3ec7c52fbe92ee14U,
  0xbdbe5caaU, 0xbd81055700000000U, 0xbed43085U, 0x0U,
  0xbdc27b2d00000000U, 0xbf2084f2U, 0xbc908f57beb37517U, 0x3efbb9f4U,
  0x3ed37317be5f6d77U, 0xbc9464183e8a4b6bU, 0x3eabe7613d827de9U, 0x3db51c043da67e84U,
  0xbd8a6da43db9ca1cU, 0x3c24c8203da4f51eU, 0x3dbe8685bf67c5b3U, 0x3e19604a3f38ac4cU,
  0xbd31a7703e05f69cU, 0x3e463196be354004U, 0xbecb36c43dbdfa79U, 0x3dbf824c3eacef7dU,
  0x3e0a85fdbe065c50U, 0xbe5025f2bf9204edU, 0x3e4d0ddabe49b1a4U, 0xbe6dc1cfbf5151afU,
  0x3ec5d5c93e3e4bbeU, 0xbec82fbe3e07370eU, 0xbed7aec3bd2801a0U, 0x3d894884bda76cc8U,
  0xbe1a2218be3f8908U, 0xbea5bcbc3aba7700U, 0x3ece1e4d3e7de56eU, 0x3ec1f453bd304af8U,
  0x3ed5f4cbbe49a5beU, 0x3df4e6b53da061d4U, 0x3e8adc5bbe36a000U, 0x3e975d43bdf107b8U,
  0x3e37f2cbbe22b694U, 0x3ac40c00bfd2ad6aU, 0xbe1fa0ae3c8bd403U, 0x3e8616fdbf60c147U,
  0xbea91dd0bda47d3cU, 0xbeac7a583e4806e2U, 0x3dd72e943e745122U, 0xbece1f44bd028c40U,
  0xbeb3c7833e4c0396U, 0xbe857819be48465bU, 0x3d9774c4bd1cc310U, 0x3db25bacbe557a4aU,
  0x3ece801d3ea84562U, 0xbdbc679bbe491e2fU, 0x3caa46f03ebcf491U, 0xbd8dd8b8be582fe5U,
  0x3e9e6c8a3e447d92U, 0xbdc3fa783fbb3887U, 0xbd9abb70bde54f93U, 0x3cf1a8103f1190ecU,
  0xbe080294be82a85dU, 0x3ea066183e57593aU, 0x3ed6e157bbc714a2U, 0xbdffca58bedd2bc1U,
  0xbe8b6f513e84e33dU, 0xbc35da803eba7eb9U, 0xbd19bf90be9865b7U, 0xbdecf984bc042060U,
  0xbecde471bed3a8fdU, 0x3e87a6783ed90171U, 0xbe324b46beb2d62cU, 0xbd6f8568be212a12U,
  0xbe1fd46d3df2addcU, 0x3e6d7382bdf0fac8U, 0x3ea2ab89be41e391U, 0xbe0f42dabe32c9d2U,
  0x3e978c87bd60d0c6U, 0x3d5041033e28da86U, 0x3ebd7195bd746f5fU, 0x3e10fbfa3e154e8aU,
  0x3eb96f8ebdf277f0U, 0x3e81bd93bf89cf23U, 0x3e5ea9aebe35a592U, 0xbec6651abf7c5732U,
  0x3ea506c33db5bbcbU, 0x3e4721b2bea741a2U, 0xbeb3d15d3e78872dU, 0x3ec69c41be99952cU,
  0xbf330ac6bd802744U, 0xbeb418533ec7344dU, 0xbe9ca682bea712b5U, 0x3eb1b8513e4f0cebU,
  0x3c79ee60bdff15b2U, 0x3e85d583be02793eU, 0xbc4a39c0be6ef090U, 0x3e53413a3d0f0f90U,
  0xbebdfc803ea5e89bU, 0x3ec8f151bd819b20U, 0x3e233b4a3e1037c3U, 0xbe8f9cba3e1b208fU,
  0x3e832edf3eb51268U, 0xbec5e582ba625400U, 0xbdcfef383e06d7fbU, 0xbe3edcf73ca26cb0U,
  0x3e5ed741bd588698U, 0xbe363482bd43a731U, 0x3edc15f0bf19f8c0U, 0x3dd22b9c3f48db1eU,
  0x3da7f59cbdf46bccU, 0xbdcfe7983e7d7122U, 0xbb35ed003ed468e1U, 0x3e1f7b7e3de05fccU,
  0xbeb285f23e5a3542U, 0xbd1b0be83eb033b7U, 0xbedad011bd2d5e80U, 0xbd15ab20bd93001cU,
  0x3dc8d1643e32b07eU, 0xbe7c90213debde8cU, 0xbe41b05d3e0ac826U, 0x3eda292b3e9f5939U,
  0xbedd5a4abec67ac7U, 0x3d082af03e5f401aU, 0x3d6aea38be58b960U, 0xbdd84b64ba938d00U,
  0xbdb63d543cace6fdU, 0xbe990be6bececbc4U, 0x3d9ee64cbc613297U, 0x3cf433903e8d5ebbU,
  0xbf5d8d713eb06285U, 0xbebfda273e35fb52U, 0x3d96c46c3eb5e385U, 0xbbd02bc03f22e77cU,
  0x3e8aef95be94a2d8U, 0xbdf3bd2d3e8b29a9U, 0xbe99878c3e6a01f8U, 0xbecf016f3dc1731cU,
  0xbe91dd29bdc3a548U, 0xbeb70dad3e2d14c1U, 0x3e4ec7f63e965ee1U, 0x3e4913debc1fd220U,
  0x3e9e2b2c3e887898U, 0x3e9870bf00000000U, 0xbe81344300000000U, 0xbc99176cbcfe2072U,
  0x3e07be9f3e8cf167U, 0x3eb0572abdf0c5f8U, 0x0U, 0xbd91d5853e8383fdU,
  0xbe9ed4cc3eb02fd1U, 0xbeec3dd2bec9c440U, 0x3f0305acbeba88baU, 0xbf0fe8d1bcaf8e9cU,
  0xbf7cf634bf08a04fU, 0x3f50f7ac3ee93b48U, 0xbcea29003e6b7750U, 0x3eae9517bf70b4c6U,
  0x3e6c630fbe8f2e88U, 0xbd876130ba064c00U, 0xbefbbf94be3cf164U, 0x3ef65bf93efd12f2U,
  0xbebad46bbe92c5b5U, 0xbe050776bf12e02eU, 0x3efe6ab4bddb9790U, 0x3e7391c63e94be2eU,
  0xbeb4ef6aU,
};


ai_handle g_sine_model_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_sine_model_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

