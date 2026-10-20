# G474_Orion_main

OrionのMain基板（STM32G474）のファームウェア。

- [システム共通仕様](specs/README.md)：通信契約、機器の責務、停止条件、FW更新。
- [Main開発文書](doc/overview.md)：内部制御、ハードウェア実装、ビルド・書込み、操作・診断。
- [仕様の管理とsubmodule運用](doc/specs.md)：正本の所在と参照方向、取得・更新手順。
- [VS Codeの設定](doc/vscode.md)：補完・定義ジャンプ・参照検索。

共通仕様は`specs/`、Main内部の実装・操作は`doc/`で管理する。実機検証結果と変更経緯は`doc/history/`に分離する。
