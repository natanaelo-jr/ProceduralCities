#version 330 core
in vec3 Normal;
in vec3 FragPos;
in vec3 Color; 

out vec4 FinalColor;

uniform vec3 viewPos; 

// COR DO CÉU (Roxo Crepúsculo)
vec3 skyColor = vec3(0.25, 0.20, 0.30); 

void main() {
    // --- 1. LUZ DIRETA (SOL) ---
    vec3 sunDir = normalize(vec3(-0.8, 0.2, -0.5)); // Sol baixo
    vec3 sunColor = vec3(1.0, 0.6, 0.3); // Laranja
    float sunIntensity = 1.5;
    float diff = max(dot(Normal, sunDir), 0.0);
    vec3 directLight = sunColor * diff * sunIntensity;

    // --- 2. LUZ HEMISFÉRICA (CÉU/SOMBRA) ---
    // Essa luz vem "de cima", mas deixa as laterais escuras
    vec3 shadowColor = vec3(0.10, 0.05, 0.15); 
    float ambientFactor = 0.5 + 0.5 * Normal.y;
    vec3 hemiLight = shadowColor * ambientFactor;

    // --- 3. LUZ AMBIENTE GLOBAL (SEU PEDIDO) ---
    // Essa é a luz "mínima" que existe em todo lugar.
    // Valor baixo (0.03 a 0.05) para não lavar o contraste.
    // Levemente roxa para combinar com o clima.
    vec3 globalAmbient = vec3(0.04, 0.03, 0.05); 

    // --- 4. RIM LIGHT (LUZ DE BORDA) ---
    // Ajuda a destacar a silhueta contra o fundo escuro
    vec3 viewDir = normalize(viewPos - FragPos);
    float rim = 1.0 - max(dot(viewDir, Normal), 0.0);
    rim = smoothstep(0.6, 1.0, rim) * 0.2; // 0.2 de força

    // --- COMBINAÇÃO ---
    // Escurece paredes laterais para dar volume
    float sideDarkening = (abs(Normal.y) < 0.9) ? 0.7 : 1.0;

    // Soma tudo: Direta + Hemisférica + Borda + GLOBAL
    vec3 lighting = (directLight + hemiLight) * sideDarkening;
    lighting += vec3(rim);         // Adiciona brilho na borda
    lighting += globalAmbient;     // Adiciona a luz mínima (base)

    // Aplica na cor do objeto
    vec3 objectColor = Color * lighting; 

    // --- 5. FOG ---
    float distance = length(FragPos - viewPos);
    float fogDensity = 0.00010; 
    float fogFactor = 1.0 / exp(pow(distance * fogDensity, 2.0));
    fogFactor = clamp(fogFactor, 0.0, 1.0);

    vec3 horizonColor = vec3(0.4, 0.3, 0.35); 
    vec3 finalFogColor = mix(skyColor, horizonColor, 0.5);
    
    vec3 finalResult = mix(finalFogColor, objectColor, fogFactor);
    
    // --- 6. PÓS-PROCESSAMENTO ---
    float exposure = 0.9; 
    finalResult = vec3(1.0) - exp(-finalResult * exposure); // Tone Mapping
    finalResult = pow(finalResult, vec3(1.0 / 2.2)); // Gamma
    
    // Um toque final de contraste
    finalResult = smoothstep(0.02, 0.98, finalResult);

    FinalColor = vec4(finalResult, 1.0);
}
