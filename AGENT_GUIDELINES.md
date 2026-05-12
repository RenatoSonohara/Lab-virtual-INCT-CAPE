# Guidelines do Agente de IA — Projeto Lab Virtual

## Objetivo
Documento curto com regras e práticas para o agente de IA que auxilia no desenvolvimento, testes e operação deste projeto.

## Escopo do Agente
- Auxiliar alterações de código, refatorações e pequenas implementações.
- Gerar e atualizar testes automatizados (unitários/integracao/e2e) quando solicitado.
- Fazer verificações de consistência (links, recursos estáticos, imports) e propor correções.
- Não executar comandos que modifiquem sistemas externos sem aprovação explícita do mantenedor.

## Regras de Segurança e Privacidade
- Não expor segredos, chaves ou credenciais em commits ou arquivos gerados.
- Rejeitar pedidos que gerem conteúdo sensível, ilegal ou perigoso.
- Quando um pedido envolve dados pessoais, pedir autorização e explicar mitigação de privacidade.

## Prompting e Limites Operacionais
- Sempre pedir confirmação para mudanças de grande alcance (migrar bibliotecas, alterar arquitetura).
- For large or risky changes, gerar um plano (todo list) com etapas e checkpoints.
- Registrar no repositório um curto resumo das mudanças sugeridas (mensagem de commit sugerida).

## Testes e Validação
- Prioridade: unitários para lógica JS isolada; e2e para fluxos críticos (simulação, carregamento CSV).
- Recomendadas ferramentas:
  - Unit: `Jest` + `jsdom` para simular DOM
  - Integration/E2E: `Cypress` ou `Playwright`
  - Lint: `ESLint` com regras básicas (no-console, no-unused-vars)
- Escrever testes que verifiquem:
  - Inicialização segura sem elementos DOM obrigatórios (guards)
  - Funções puras de cálculo (ex.: conversões, interpolação)
  - Export/Import de CSV (parsing correto)
  - Estado da UI após ações principais (start, reset)

## Exemplo rápido de casos de teste (coletor/app.js)
- `init` não lança se elementos DOM ausentes
- `updateClock()` retorna silenciosamente quando `#clockNow` não existe
- `startSimulation()` muda `uiRunning(true)` e ativa tick loop
- `parseCsv()` parseia cabeçalho e campos numéricos corretamente

## CI / Pipeline
- Ao usar GitHub: criar workflow que rode `npm ci`, `npm test`, `npm run lint` em PRs.
- Bloquear merge se cobertura mínima não for atendida (ex.: 60% para arquivos alterados).

## Observabilidade e Logs
- Padronizar logs de runtime para `console.debug/info/warn/error` com mensagens curtas.
- Adicionar um mecanismo simples para coletar erros JS em produção (window.onerror → enviar a endpoint ou salvar localmente durante testes).

## Estratégias de Escala e Estabilidade
- Modularizar código JS em módulos pequenos e testáveis (mover lógica pesada de `coletor/app.js` para `src/`)
- Introduzir bundle/tooling (Rollup/Webpack/Vite) se crescimento justificar — melhora testes e importação de módulos
- Implantar testes e lint em CI; rodar e2e periodicamente (nightly) para detectar regressões
- Adotar versionamento semântico e changelogs automáticos (Dependabot para deps)
- Monitoramento: usar Sentry/LogRocket para erros JS em produção (opcional)

## Operacional do Agente
- Sempre atualizar o arquivo `AGENT_GUIDELINES.md` quando novas práticas forem adotadas.
- Antes de modificar arquivos críticos, gerar um diff simulado e pedir aprovação.

## Roadmap sugerido (curto prazo)
1. Scaffold de testes unitários com `Jest` e um `package.json` básico
2. Escrever 10-15 testes unitários cobrindo funções críticas do `coletor`
3. Configurar GitHub Actions para rodar testes em PRs
4. Adicionar 2 e2e com `Cypress` cobrindo fluxo de simulação e upload CSV

---

Arquivo gerado automaticamente pelo agente — revise e ajuste preferências (ferramentas, cobertura mínima, política de merges).