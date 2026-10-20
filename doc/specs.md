# 統合仕様リポジトリ

複数レイヤ・リポジトリにまたがる仕様は、[ibis-ssl/specs](https://github.com/ibis-ssl/specs) を正本とする。本リポジトリではルートの `specs/` に Git submodule として配置する。

- [仕様の目次](../specs/README.md)
- [共通仕様](../specs/common/)
- [Main 基板の対外仕様](../specs/README.md#orionの対外仕様)

## 正本と参照方向

| 情報 | 正本 |
|---|---|
| 機器間の通信形式・単位・責務・停止条件・更新契約 | `specs/` |
| Main内部の計算・状態・GPIO・割り込み・ビルド・操作・診断 | 本repoの[開発文書](overview.md) |
| 実機測定・変更理由・過去のレビュー | 本repoの[履歴](history/README.md) |

```text
各repoの実装文書 → specsの共通契約 → specs内の他の共通定義
specsの目次・関連仕様本文 → 各repoの実装・操作の詳細
```

上段は仕様の依存方向、下段は読者のための案内リンクである。相互リンクがあっても正本を相互依存させない。共通仕様がMain内のゲインやCLI操作を定義する文書へ戻って契約を決める構成にはしない。

共通契約はspecsで1か所に定義し、各repoから参照する。Main内部の説明は本repoで1か所に定義する。同じ事実を両側の本文で保守しない。案内だけの個別文書は作らず、目次または関連仕様から正本へ直接リンクする。コードと契約が食い違う場合は適合済みと扱わず、差と影響を実装文書に明記する。現行コードへ合わせるだけの目的で共通契約を変更しない。

新たな共通インターフェイスはspecsの作業ブランチで定義し、関係repoの実装・結合確認を揃える。内部だけの変更は本repoで完結させる。未実装案は`specs/proposals/`、日付付きの作業・検証は`doc/history/`へ置く。履歴にある旧仕様や旧手順を現在の正本として使用しない。

## 取得

新規 clone 時は submodule も取得する。

```sh
git clone --recurse-submodules https://github.com/ibis-ssl/G474_Orion_main.git
```

既存の clone や親リポジトリの更新後は、親リポジトリが記録するコミットを取得する。

```sh
git submodule update --init --recursive
```

## 参照する仕様の更新

親リポジトリは `specs` のコミットを固定して参照する。統合仕様の既定ブランチの最新版へ更新する場合は、次を実行して内容を確認し、参照コミットの変更を親リポジトリでコミットする。

```sh
git submodule update --remote specs
git diff --submodule=log
git add specs
```

## specsの編集

`specs/`は独立したGit repoなので、そこで作業ブランチを作る。

```sh
git -C specs switch -c docs/<topic>
git -C specs diff --check
git -C specs diff
```

レビュー後は先にspecs側でコミット・pushし、その公開済みコミットを親repoのsubmodule参照に記録する。親repoだけをコミットしてもspecs内の未コミット本文は配布されない。ブランチの作成・文書編集と、コミット・pushは別の操作である。
